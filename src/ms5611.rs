// address
pub const MS5611_ADDRESS: u8 = 0x77;  // when CSB is low

// commands
pub const RESET_CMD: u8 = 0x1E;
pub const PROM_READ_CMD: u8 = 0xA0;
pub const ADC_READ_CMD: u8 = 0x00;
pub const CONVERT_PRESSURE_CMD: u8 = 0x40;
pub const CONVERT_TEMP_CMD: u8 = 0x50;


// PROM calibration data struct
pub struct PromData {
    pub c1: u16,
    pub c2: u16,
    pub c3: u16,
    pub c4: u16,
    pub c5: u16,
    pub c6: u16,
}

pub fn ms5611_crc4(prom: &[u16]) -> u16 {
    let mut n_rem: u16 = 0x00;

    let mut prom_bytes = [0u8; 16];
    for (i, word) in prom.iter().enumerate() {
        prom_bytes[2 * i] = (word >> 8) as u8;
        prom_bytes[2 * i + 1] = *word as u8;
    }

    prom_bytes[15] = 0; // CRC byte must be replaced with zero during calculation

    for i in 0..16 {
        n_rem ^= (prom_bytes[i] as u16) << 8;
        for _ in 0..8 {
            if (n_rem & 0x8000) != 0 {
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem = n_rem << 1;
            }
        }
    }

    (n_rem >> 12) & 0xF // final 4-bit CRC
}

pub fn compensate_ms5611(d1: u32, d2: u32, cal: &PromData) -> (f32, f32) {
    let delta_t = d2 as i64 - ((cal.c5 as i64) << 8);
    let temp = 2000 + ((delta_t * cal.c6 as i64) >> 23);

    let off = ((cal.c2 as i64) << 16) + ((cal.c4 as i64 * delta_t) >> 7);
    let sens = ((cal.c1 as i64) << 15) + ((cal.c3 as i64 * delta_t) >> 8);

    let pressure = (((d1 as i64 * sens) >> 21) - off) >> 15;

    (
        temp as f32 / 100.0,       // °C
        pressure as f32 / 100.0,   // mbar
    )
}
