#![no_std]
#![no_main]

mod sensors;
mod lsm6dsox;
mod registers;
mod usb_serial;

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
use embassy_time::{Timer};
use static_cell::StaticCell;

// Sensors
use crate::sensors::*;

// USB
use crate::usb_serial::*;

static I2C_BUS: StaticCell<NoopMutex<RefCell<I2c<'static, Async>>>> = StaticCell::new();

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

    let class = setup_usb(p.USB, p.PA12, p.PA11).await;
    
    spawner.spawn(lsm6dsox_task(i2c_lsm6dsox)).unwrap();
    
    
    loop {
        led.set_high();
        Timer::after_millis(500).await;
        led.set_low();
        Timer::after_millis(500).await;
    }
}