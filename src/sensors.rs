use defmt::*;
use embassy_stm32::i2c::I2c;
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::USB;
use embassy_stm32::usb::Driver;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Timer, Delay, Duration};
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embedded_hal::prelude::{_embedded_hal_blocking_i2c_Write, _embedded_hal_blocking_i2c_WriteRead};
use {defmt_rtt as _, panic_probe as _};
use crate::lsm6dsox::*;
use crate::registers::*;
use crate::usb_serial::usb_print;
use crate::usb_write;

pub const DEVID: u8 = 0x00;
pub const RESET_CMD: u8 = 0x1E;
pub const PROM_READ_CMD: u8 = 0xA0;
pub const ADC_READ_CMD: u8 = 0x00;
pub const CONVERT_PRESSURE_CMD: u8 = 0x40;
pub const CONVERT_TEMP_CMD: u8 = 0x50;

const LSM6DSOX_ADDRESS: u8 = 0x6A;  // LSM6DSOX when SA0 is low
const IIS2MDCTR_ADDRESS: u8 = 0x1E;  // fixed since no SA0 pin
const MS5611_ADDRESS: u8 = 0x77;  // when CSB is low

#[embassy_executor::task]
pub async fn lsm6dsox_task(mut i2c: I2cDevice<'static, NoopRawMutex, I2c<'static,  Async>>, class: &'static mut CdcAcmClass<'static, Driver<'static, USB>>,) {
    // Set up sensor (accelerometer and gyroscope configuration) before the loop
    setup_lsm6dsox(&mut i2c).await;

    let accel_scale = AccelerometerScale::Accel16g; // Set this as needed (e.g., Accel16g, Accel4g, Accel8g)
    let gyro_scale = GyroscopeScale::Dps2000; // Set this as needed (e.g., Dps125, Dps500, Dps1000, Dps2000)

    loop {
        // Check if new accelerometer data is ready
        let mut status = [0u8; 1];
        match i2c.write_read(LSM6DSOX_ADDRESS, &[STATUS_REG], &mut status) {
            Ok(()) => {
                // If the XLDA (accelerometer data available) bit is not set, continue to the next loop iteration
                if (status[0] & 0b0000_0001) == 0 {
                    continue; // No new data, skip this iteration
                }
            }
            Err(_) => {
                error!("Failed to read STATUS_REG");
                continue;
            }
        }
        
        // Read accelerometer data (6 bytes: 2 for each axis)
        let mut accel_data = [0u8; 6];
        match i2c.write_read(LSM6DSOX_ADDRESS, &[OUTX_L_A], &mut accel_data) {
            Ok(()) => {
                // Combine low and high bytes for X, Y, and Z axes using manual little-endian conversion
                let x_raw = i16::from_le_bytes([accel_data[0], accel_data[1]]) as f32;
                let y_raw = i16::from_le_bytes([accel_data[2], accel_data[3]]) as f32;
                let z_raw = i16::from_le_bytes([accel_data[4], accel_data[5]]) as f32;

                // Convert raw accelerometer data to engineering units (e.g., m/s²)
                let scale = accel_scale.to_factor();
                let x_m_s2 = x_raw * scale;
                let y_m_s2 = y_raw * scale;
                let z_m_s2 = z_raw * scale;

                //info!("Raw Accelerometer Data (m/s²): X = {}, Y = {}, Z = {}", x_raw, y_raw, z_raw);
                info!("Accelerometer Data (G): X = {}, Y = {}, Z = {}", x_m_s2, y_m_s2, z_m_s2);
                usb_write!(class, "Accel: X={}, Y={}, Z={}", x_m_s2, y_m_s2, z_m_s2);
            }
            Err(_) => {
                error!("Failed to read accelerometer data");
            }
        }

        // Read gyroscope data for X, Y, Z axes (6 bytes: 2 for each axis)
        let mut gyro_data = [0u8; 6];
        match i2c.write_read(LSM6DSOX_ADDRESS, &[OUTX_L_G], &mut gyro_data) {
            Ok(()) => {
                // Combine low and high bytes for X, Y, and Z axes with Little Endian handling
                let x_raw = i16::from_le_bytes([gyro_data[0], gyro_data[1]]) as f32;
                let y_raw = i16::from_le_bytes([gyro_data[2], gyro_data[3]]) as f32;
                let z_raw = i16::from_le_bytes([gyro_data[4], gyro_data[5]]) as f32;
        
                // Convert raw gyroscope data to engineering units (e.g., degrees per second)
                let scale = gyro_scale.to_factor();
                let x_dps = x_raw * scale;
                let y_dps = y_raw * scale;
                let z_dps = z_raw * scale;
        
                info!("Gyroscope Data (°/s): X = {}, Y = {}, Z = {}", x_dps, y_dps, z_dps);
            }
            Err(_) => {
                error!("Failed to read gyroscope data");
            }
        }

        // Wait for a short period before the next loop iteration
        Timer::after(Duration::from_millis(200)).await;
    }
}

pub async fn setup_lsm6dsox(i2c: &mut I2cDevice<'static, NoopRawMutex, I2c<'static, Async>>) {
    // 1. reset software
    let reset_command: u8 = 0x01; // SW Reset Command for LSM6DSOX (CTRL3_C)
    if update_reg_address(i2c, LSM6DSOX_ADDRESS, CTRL3_C, 0b0000_0001, reset_command).await.is_err() {
        error!("Failed to send sensor reset command");
    }
    
    // 2. wait for software to reset
    let mut ctrl3_c_val = [0u8; 1];
    for _ in 0..5 {
        Timer::after(Duration::from_millis(10)).await;
        match i2c.write_read(LSM6DSOX_ADDRESS, &[CTRL3_C], &mut ctrl3_c_val) {
            Ok(()) => {
                if ctrl3_c_val[0] & 0x01 == 0 {
                    break;
                }
            }
            Err(_) => {
                error!("Failed to read CTRL3_C during reset check");
            }
        }
    }

    // 3. Disable I3C and enable block data update using update_reg_address
    if update_reg_address(i2c, LSM6DSOX_ADDRESS, CTRL9_XL, 0b0000_0011, 0x03).await.is_err() {
        error!("Failed to configure I3C interface");
    }
    if update_reg_address(i2c, LSM6DSOX_ADDRESS, I3C_BUS_AVB, 0b0001_1000, 0b0001_1000).await.is_err() {
        error!("Failed to configure I3C Bus AVB");
    }
    if update_reg_address(i2c, LSM6DSOX_ADDRESS, CTRL3_C, 0b0100_0000, 0b0100_0000).await.is_err() {
        error!("Failed to enable Block Data Update");
    }

    // 4. Set accel and gyro scale using update_reg_address
    let accel_odr = DataRate::Freq6660Hz as u8;  // 0xA0
    let accel_scale = AccelerometerScale::Accel16g as u8;  // 0x04
    let accel_config = accel_odr | accel_scale;
    info!("Accel config: {:#X}", accel_config);
    if update_reg_address(i2c, LSM6DSOX_ADDRESS, CTRL1_XL, 0b1111_1100, accel_config).await.is_err() {
        error!("Failed to configure CTRL1_XL");
    }

    let gyro_odr = DataRate::Freq6660Hz as u8;
    let gyro_scale = GyroscopeScale::Dps2000 as u8;
    let gyro_config = gyro_odr | gyro_scale;
    if update_reg_address(i2c, LSM6DSOX_ADDRESS, CTRL2_G, 0b1111_1100, gyro_config).await.is_err() {
        error!("Failed to configure CTRL2_G");
    }
}


pub async fn check_iis2mdctr(mut i2c: I2cDevice<'static, NoopRawMutex, I2c<'static, Async>>) {
    let mut data = [0u8; 1];
    info!("Checking IIS2MDCTR");

    match i2c.write_read(LSM6DSOX_ADDRESS, &[0x4F], &mut data) {
        Ok(()) => {
            info!("Whoami: 0x{:02x}", data[0]);
        }
        Err(_) => error!("IIS2MDCTR check failed")
    }
}

pub async fn check_ms5611(mut i2c: I2cDevice<'static, NoopRawMutex, I2c<'static, Async>>) {
    let mut data = [0u8; 12];
    info!("Checking ms5611");

    info!("Resetting ms5611");
    match i2c.write(MS5611_ADDRESS, &[RESET_CMD]) {
        Ok(()) => {
            info!("Reset Success");
            Timer::after_millis(3).await;
        }
        Err(_) => error!("Reset Failed")
    }
    
    info!("Reading PROM data");
    for i in 0..6 {
        match i2c.write_read(MS5611_ADDRESS, &[PROM_READ_CMD + (i as u8) * 2], &mut data[i*2..(i+1)*2]) {
            Ok(()) => {}
            Err(_) => error!("Failed to get PROM data")
        }
    }
    info!("PROM calibration data: {:02x}", data);

    // // Step 3: Start pressure conversion
    // unwrap!(i2c.blocking_write(MS5611_ADDRESS, &[CONVERT_PRESSURE_CMD]));
    // Timer::after_millis(10).await; // Wait for conversion (depending on the OSR setting)
    // 
    // // Step 4: Read ADC result for pressure
    // unwrap!(i2c.blocking_write_read(MS5611_ADDRESS, &[ADC_READ_CMD], &mut data));
    // let pressure_adc_result = u32::from(data[0]) << 16 | u32::from(data[1]) << 8 | u32::from(data[2]);
    // info!("Pressure ADC result: 0x{:06x}", pressure_adc_result);
    // 
    // // Step 5: Start temperature conversion
    // unwrap!(i2c.blocking_write(MS5611_ADDRESS, &[CONVERT_TEMP_CMD]));
    // Timer::after_millis(10).await; // Wait for conversion (depending on the OSR setting)
    // 
    // // Step 6: Read ADC result for temperature
    // unwrap!(i2c.blocking_write_read(MS5611_ADDRESS, &[ADC_READ_CMD], &mut data));
    // let temp_adc_result = u32::from(data[0]) << 16 | u32::from(data[1]) << 8 | u32::from(data[2]);
    // info!("Temperature ADC result: 0x{:06x}", temp_adc_result);
}

pub async fn check_adxl375(mut i2c: I2cDevice<'static, NoopRawMutex, I2c<'static, Async>>, address: u8) {
    let mut data = [0u8; 1];
    if (address == 0x1D) {
        info!("Checking ADXL375 1");
    }
    else if (address == 0x53) { 
        info!("Checking ADXL375 2");
    }

    match i2c.write_read(address, &[0x00], &mut data) {
        Ok(()) => {
            info!("Whoami: 0x{:02x}", data[0]);
        }
        Err(_) => error!("ADXL375 check failed")
    }
}

pub async fn check_lis3dh(mut i2c: I2cDevice<'static, NoopRawMutex, I2c<'static, Async>>, address: u8) {
    let mut data = [0u8; 1];
    if (address == 0x19) {
        info!("Checking LIS3DH 1");
    }
    else if (address == 0x18) {
        info!("Checking LIS3DH 2");
    }

    match i2c.write_read(address, &[0x0F], &mut data) {
        Ok(()) => {
            info!("Whoami: 0x{:02x}", data[0]);
        }
        Err(_) => error!("LIS3DH check failed")
    }
}
