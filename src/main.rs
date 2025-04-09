#![no_std]
#![no_main]

mod sensors;
mod lsm6dsox;
mod registers;
mod usb_serial;
mod adxl375;
mod lis3dh;

use core::cell::RefCell;
use defmt::*;
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, i2c, peripherals, Config};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Async;
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex, ThreadModeRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::{PubSubBehavior, PubSubChannel};
use embassy_time::{Timer};
use static_cell::StaticCell;
use crate::adxl375::{ADXL375_LOW_ADDRESS, ADXL375_HIGH_ADDRESS};
use crate::lis3dh::{LIS3DH_HIGH_ADDRESS, LIS3DH_LOW_ADDRESS};
// Sensors
use crate::sensors::*;

// USB
use crate::usb_serial::*;

static I2C_BUS: StaticCell<NoopMutex<RefCell<I2c<'static, Async>>>> = StaticCell::new();

// Combine all sensor data into one struct for packetization
#[derive(Clone)]
pub struct SensorPacket {
    pub lsm_accel: AccelData,
    pub gyro: GyroData,
    pub pressure: f32,
    pub temperature: f32,
    pub adxl_1: AccelData,
    pub adxl_2: AccelData,
    pub lis_1: AccelData,
    pub lis_2: AccelData,
}

pub const SENSOR_SUBSCRIBERS: usize = 3; // how many subscribers to this channel
pub const SENSOR_PUBLISHERS: usize = 1;  // how many publishers (senders)
pub static SENSOR_PUBSUB: PubSubChannel<CriticalSectionRawMutex, SensorPacket, 8, SENSOR_SUBSCRIBERS, SENSOR_PUBLISHERS> = PubSubChannel::new();

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

// #[embassy_executor::task]
// async fn check_sensors(i2c: I2cDevice<'static, NoopRawMutex, I2c<'static, Async>>) {
//     check_lsm6dsox(i2c).await; // check the lsm6dsox whoami
// }

// Main function and entry point of the program after configuration
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = true;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI, // 16 MHz
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL10,
            divp: None,
            divq: None,
            divr: Some(PllDiv::DIV1), // 160 MHz
        });
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.voltage_range = VoltageScale::RANGE1;
        config.rcc.hsi48 = Some(Hsi48Config { sync_from_usb: true }); // needed for USB
        config.rcc.mux.iclksel = mux::Iclksel::HSI48; // USB uses ICLK (48MHz)
    }

    // Setup peripherals on the default clock
    let p = embassy_stm32::init(config);

    // Set up user led as output
    let mut led = Output::new(p.PE2, Level::Low, Speed::Medium);
    
    // Use I2C1 for sensors with default clock
    let i2c = I2c::new(p.I2C1, p.PB6, p.PB3, Irqs, p.GPDMA1_CH4, p.GPDMA1_CH5, Hertz(100_000), Default::default());
    
    let i2c_bus = NoopMutex::new(RefCell::new(i2c));
    let i2c_bus = I2C_BUS.init(i2c_bus);
    
    let i2c_lsm6dsox = I2cDevice::new(i2c_bus);
    let i2c_iis2mdctr = I2cDevice::new(i2c_bus);
    let i2c_ms5611 = I2cDevice::new(i2c_bus);
    let i2c_adxl375_1 = I2cDevice::new(i2c_bus);
    let i2c_adxl375_2 = I2cDevice::new(i2c_bus);
    let i2c_lis3dh_1 = I2cDevice::new(i2c_bus);
    let i2c_lis3dh_2 = I2cDevice::new(i2c_bus);
    
    spawner.spawn(lsm6dsox_task(i2c_lsm6dsox)).unwrap();
    spawner.spawn(adxl375_task(i2c_adxl375_1, ADXL375_LOW_ADDRESS)).unwrap();
    spawner.spawn(adxl375_task(i2c_adxl375_2, ADXL375_HIGH_ADDRESS)).unwrap();
    spawner.spawn(lis3dh_task(i2c_lis3dh_1, LIS3DH_LOW_ADDRESS)).unwrap();
    spawner.spawn(lis3dh_task(i2c_lis3dh_2, LIS3DH_HIGH_ADDRESS)).unwrap();
    
    spawner.spawn(aggregator_task()).unwrap();
    
    setup_usb(p.USB, p.PA12, p.PA11, &SENSOR_PUBSUB).await;
    
    loop {
        led.set_high();
        Timer::after_millis(500).await;
        led.set_low();
        Timer::after_millis(500).await;
    }
}

#[embassy_executor::task]
async fn aggregator_task() {
    fn try_receive_to<T: core::fmt::Debug>(
        channel: &embassy_sync::channel::Channel<CriticalSectionRawMutex, T, 8>,
        dest: &mut T,
        label: &str,
    ) {
        match channel.try_receive() {
            Ok(data) => *dest = data,
            Err(e) => info!("Error receiving {} data: {:?}", label, e),
        }
    }
    
    loop {
        let mut data_packet = SensorPacket {
            lsm_accel: AccelData::default(),
            gyro: GyroData::default(),
            pressure: 0.0,
            temperature: 0.0,
            adxl_1: AccelData::default(),
            adxl_2: AccelData::default(),
            lis_1: AccelData::default(),
            lis_2: AccelData::default(),
        };
        try_receive_to(&LSM_ACCEL_CHANNEL, &mut data_packet.lsm_accel, "LSM accel");
        try_receive_to(&GYRO_CHANNEL, &mut data_packet.gyro, "gyro");
        try_receive_to(&ADXL375_1_CHANNEL, &mut data_packet.adxl_1, "ADXL1");
        try_receive_to(&ADXL375_2_CHANNEL, &mut data_packet.adxl_2, "ADXL2");
        try_receive_to(&LIS3DH_1_CHANNEL, &mut data_packet.lis_1, "LIS1");
        try_receive_to(&LIS3DH_2_CHANNEL, &mut data_packet.lis_2, "LIS2");
        info!("Accel Data LIS: X = {}, Y = {}, Z = {}", data_packet.lis_2.x, data_packet.lis_2.y, data_packet.lis_2.z);
        
        // Broadcast this packet to telemetry consumers (data storage, radio, CAN)
        // This will publish without waiting for an empty slot. change to a publisher to correctly wait for space with "publish()"
        SENSOR_PUBSUB.publish_immediate(data_packet);
        
        // adjust timer based on how fast each subscriber needs the data
        Timer::after_millis(100).await;
    }
}
