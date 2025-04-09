use defmt::{panic, *};
use defmt_rtt as _; // global logger
use embassy_futures::join::join;
use embassy_stm32::usb::{DmPin, DpPin, Driver, Instance};
use embassy_stm32::{bind_interrupts, peripherals, usb, Peri};
use embassy_stm32::peripherals::{USB};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use panic_probe as _;
use core::fmt::{Write};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, WaitResult};
use heapless::String;
use crate::{SensorPacket};

bind_interrupts!(struct Irqs {
    USB => usb::InterruptHandler<peripherals::USB>;
});

async fn usb_print<'a, T: Instance>(
    class: &mut CdcAcmClass<'a, Driver<'a, T>>,
    args: core::fmt::Arguments<'_>,
) {
    let mut buffer: String<256> = String::new(); // adjust size as needed
    let _ = buffer.write_fmt(args);
    
    // Send in chunks of 64 or less
    let mut i = 0;
    while i < buffer.len() {
        let end = (i + 64).min(buffer.len());
        let chunk = &buffer.as_bytes()[i..end];
        if let Err(e) = class.write_packet(chunk).await {
            warn!("USB write error: {:?}", e);
            break;
        }
        i = end;
    }

    // If last chunk was exactly 64 bytes, send a ZLP
    if buffer.len() % 64 == 0 {
        let _ = class.write_packet(&[]).await;
    }
}

// Handy macro
#[macro_export]
macro_rules! usb_write {
    ($usb:expr, $($arg:tt)*) => {
        usb_print($usb, format_args!($($arg)*)).await
    };
}

pub async fn setup_usb<'d>(
    usb: Peri<'d, USB>,
    dp: Peri<'d, impl DpPin<USB>>,
    dm: Peri<'d, impl DmPin<USB>>,
    pub_sub_channel: &PubSubChannel<CriticalSectionRawMutex, SensorPacket, 8, 3, 1>,
) {
    
    // Do not need the vbus protection config since it doesn't exist for this chip?
    let driver = Driver::new(usb, Irqs, dp, dm);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-serial example");
    config.serial_number = Some("12345678");

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();
    
    // Create the subscriber for the usb_serial
    let mut usb_subscriber = pub_sub_channel.subscriber().unwrap();
    
    // Do stuff with the class!
    let echo_fut = async {
        loop {
            class.wait_connection().await;
            info!("USB Connected");
            usb_write!(&mut class, "lsm_accelx,lsm_accely,lsm_accelz,gyrox,gyroy,gyroz,pressure,temp\r\n");
            
            // Continuously poll for sensor data
            loop {
                // Check if there are any new sensor messages
                match usb_subscriber.next_message().await {
                    WaitResult::Message(packet) => {
                        usb_write!(&mut class, "{}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}\r\n",
                        packet.lsm_accel.x, packet.lsm_accel.y, packet.lsm_accel.z,
                        packet.gyro.x, packet.gyro.y, packet.gyro.z,
                        packet.pressure, packet.temperature,
                        packet.adxl_1.x, packet.adxl_1.y, packet.adxl_1.z,
                        packet.adxl_2.x, packet.adxl_2.y, packet.adxl_2.z,
                        packet.lis_1.x, packet.lis_1.y, packet.lis_1.z,
                        packet.lis_2.x, packet.lis_2.y, packet.lis_2.z,
                    );
                    }
                    WaitResult::Lagged(e) => {
                        info!("USB Lagged {:?}", e);
                    }
                }
            }
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, echo_fut).await;
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn echo<'d, T: Instance + 'd>(class: &mut CdcAcmClass<'d, Driver<'d, T>>) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        info!("data: {:x}", data);
        class.write_packet(data).await?;
    }
}

