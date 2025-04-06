use defmt::*;
use embassy_stm32::i2c::I2c;
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Timer, Delay, Duration};
use embedded_hal::prelude::{_embedded_hal_blocking_i2c_Write, _embedded_hal_blocking_i2c_WriteRead};
use {defmt_rtt as _, panic_probe as _};

/// Read-modify-write a register for a given sensor at a given address. Prevents overwriting
/// the already present register bits.
pub async fn update_reg_address(
    i2c: &mut I2cDevice<'static, NoopRawMutex, I2c<'static, Async>>,
    address: u8,
    reg: u8,
    bitmask: u8,
    new_value: u8,
) -> Result<(), ()> {
    let mut reg_val = [0u8; 1];

    // Step 1: Read the current value of the register
    match i2c.write_read(address, &[reg], &mut reg_val) {
        Ok(()) => {
            // Step 2: Log the previous register value
            info!("Register 0x{:02X} previous value: 0x{:02X}", reg, reg_val[0]);

            // Step 3: Modify the register value by clearing the bits specified by the mask
            reg_val[0] &= !bitmask; // Clear the bits we want to change

            // Step 4: Set the new data to the masked bits
            reg_val[0] |= new_value & bitmask; // Apply new value to the masked bits

            // Step 5: Log the new register value
            info!("Register 0x{:02X} new value: 0x{:02X}", reg, reg_val[0]);

            // Step 6: Write the modified value back to the register
            match i2c.write(address, &[reg, reg_val[0]]) {
                Ok(()) => {
                    info!("Register 0x{:02X} updated successfully", reg);
                    Ok(())
                },
                Err(_) => {
                    error!("Failed to write register 0x{:02X}", reg);
                    Err(())
                }
            }
        },
        Err(_) => {
            error!("Failed to read register 0x{:02X}", reg);
            Err(())
        }
    }
}
