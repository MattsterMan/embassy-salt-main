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
use heapless::String;

bind_interrupts!(struct Irqs {
    USB => usb::InterruptHandler<peripherals::USB>;
});

pub async fn setup_usb<'d>(
    usb: Peri<'d, USB>,
    dp: Peri<'d, impl DpPin<USB>>,
    dm: Peri<'d, impl DmPin<USB>>,
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

    // Do stuff with the class!
    let echo_fut = async {
        loop {
            class.wait_connection().await;
            info!("Connected");

            usb_print(&mut class, format_args!("Hello USB!\r\n")).await;
            
            let _ = echo(&mut class).await;
            info!("Disconnected");
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

pub async fn usb_print<'a, T: Instance>(
    class: &mut CdcAcmClass<'a, Driver<'a, T>>,
    args: core::fmt::Arguments<'_>,
) {
    let mut buffer: String<256> = String::new(); // adjust size as needed
    let _ = buffer.write_fmt(args);
    let _ = class.write_packet(buffer.as_bytes()).await;
}

// Handy macro
#[macro_export]
macro_rules! usb_write {
    ($usb:expr, $($arg:tt)*) => {
        usb_print($usb, format_args!($($arg)*)).await
    };
}
