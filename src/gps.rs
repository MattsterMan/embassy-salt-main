use defmt::*;
use core::fmt::Write as _;
use embassy_stm32::gpio::{Flex, Output, Pull};
use embassy_stm32::mode::{Async};
use embassy_stm32::usart::{UartRx, UartTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use heapless::{String, Vec};
use embassy_futures::select::{select, Either, Select};

pub static GPS_CHANNEL: Channel<CriticalSectionRawMutex, [u8; 256], 8> = Channel::new();

// PMTK messages
pub const PMTK_RMC_ONLY: &str = "PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";  // only output RMC
pub const PMTK_SET_POS_FIX_10HZ: &str = "PMTK220,100";  // increase output rate to 10HZ
pub const PMTK_API_SET_FIX_CTL_10HZ: &str = "PMTK300,100,0,0,0,0";  // increase fix calculation to 10HZ

#[derive(Debug)]
pub struct GpsRmc {
    /// UTC time as hhmmss (fractional seconds dropped), e.g., 123519.
    pub utc_time: Option<u32>,
    /// Date as ddmmyy, e.g., 230394.
    pub date: Option<u32>,
    /// Data validity: 'A' = valid, 'V' = invalid.
    pub status: u8,
    /// Latitude in signed decimal degrees (positive for North, negative for South).
    pub latitude: Option<f32>,
    /// Longitude in signed decimal degrees (positive for East, negative for West).
    pub longitude: Option<f32>,
    /// Speed over ground in knots.
    pub speed: Option<f32>,
    /// Course over ground in degrees.
    pub course: Option<f32>,
}

// ==============================
// Helper Parsers (Minimal)
// ==============================

/// Parse the UTC time from a string of the form "hhmmss.sss".
/// Fractional seconds are dropped; e.g. "123519.00" becomes 123519.
fn parse_time_simple(s: &str) -> Option<u32> {
    // Split at the decimal and keep the integer part only.
    let int_part = s.split('.').next()?;
    int_part.parse::<u32>().ok()
}

/// Parse the date from a string "ddmmyy" into a u32.
fn parse_date_simple(s: &str) -> Option<u32> {
    s.parse::<u32>().ok()
}

// ==============================
// RMC Parsing Function (Minimal)
// ==============================

/// Parses an RMC sentence string and returns a GpsRmc struct.
/// Returns None if the sentence is not a valid RMC sentence.
fn parse_rmc(sentence: &str) -> Option<GpsRmc> {
    // Remove checksum (if exists) by splitting at '*'
    let sentence_no_checksum = sentence.split('*').next()?;

    // Only process RMC sentences (variants may be "$GPRMC" or "$GNRMC")
    if !sentence_no_checksum.starts_with("$GPRMC") && !sentence_no_checksum.starts_with("$GNRMC") {
        return None;
    }

    // Split the sentence into its fields.
    let parts: Vec<&str, 16> = sentence_no_checksum.split(',').collect();
    if parts.len() < 12 {
        return None;
    }

    // Field indices based on the RMC sentence:
    // 0: Header, 1: UTC time, 2: Status, 3: Latitude, 4: N/S,
    // 5: Longitude, 6: E/W, 7: Speed, 8: Course, 9: Date, 
    // (fields 10 and 11 are magnetic variation info and are ignored)

    let utc_time = if !parts[1].is_empty() {
        parse_time_simple(parts[1])
    } else {
        None
    };

    let status = parts[2].bytes().next().unwrap_or(b'V');

    // Process latitude: parse and then convert to a signed coordinate.
    let latitude = if !parts[3].is_empty() {
        parts[3].parse::<f32>().ok().map(|lat| {
            if let Some(dir) = parts[4].chars().next() {
                if dir == 'S' { -lat } else { lat }
            } else {
                lat
            }
        })
    } else {
        None
    };

    // Process longitude: parse and apply sign based on direction.
    let longitude = if !parts[5].is_empty() {
        parts[5].parse::<f32>().ok().map(|lon| {
            if let Some(dir) = parts[6].chars().next() {
                if dir == 'W' { -lon } else { lon }
            } else {
                lon
            }
        })
    } else {
        None
    };

    let speed = if !parts[7].is_empty() {
        parts[7].parse::<f32>().ok()
    } else {
        None
    };

    let course = if !parts[8].is_empty() {
        parts[8].parse::<f32>().ok()
    } else {
        None
    };

    let date = if !parts[9].is_empty() {
        parse_date_simple(parts[9])
    } else {
        None
    };

    Some(GpsRmc {
        utc_time,
        date,
        status,
        latitude,
        longitude,
        speed,
        course,
    })
}

#[embassy_executor::task]
pub async fn gps_reader(mut rx: UartRx<'static, Async>) {

    let mut buf = [0u8; 128];
    let mut line = Vec::<u8, 512>::new(); // Heapless buffer to accumulate NMEA burst

    loop {
        match rx.read_until_idle(&mut buf).await {
            Ok(n) => {
                for &byte in &buf[..n] {
                    // Accumulate until newline
                    line.push(byte).ok();
                    if byte == b'\n' {
                        if let Ok(s) = core::str::from_utf8(&line) {
                            info!("GPS RX: {}", s.trim_end());
                        } else {
                            warn!("GPS RX: Invalid UTF-8");
                        }
                        line.clear();
                    }
                }
            }
            Err(e) => {
                warn!("GPS RX error: {:?}", e);
                line.clear(); // Drop partial data on error
            }
        }
    }
}

pub async fn pulse_gps_standby(mut stby_pin: Flex<'_>) {
    info!("Pulsing GPS STANDBY pin low to exit standby");
    // Step 1: Pull low
    stby_pin.set_low();
    Timer::after_millis(150).await;

    // Step 2: Release to a float
    stby_pin.set_as_input(Pull::None);

    // Step 3: Now the pin is floating (Hi-Z), GPS will go to Full On
    info!("Released STANDBY pin (floating now, GPS should wake up)");
}

pub async fn setup_gps(
    tx: &mut UartTx<'static, Async>,
    rx: &mut UartRx<'static, Async>,
    gps_stby: Flex<'_>,
) {
    // ensure not in standby mode
    pulse_gps_standby(gps_stby).await;
    
    // enable RTC only
    if let Err(e) = send_pmtk_cmd_and_wait_ack(tx, rx, PMTK_RMC_ONLY).await {
        // Handle the error, e.g., log it or take corrective action
        info!("Error: {}", e);
    }
    
    // increase message output interval
    if let Err(e) = send_pmtk_cmd_and_wait_ack(tx, rx, PMTK_SET_POS_FIX_10HZ).await {
        // Handle the error, e.g., log it or take corrective action
        info!("Error: {}", e);
    }
    
    // increase fix calculation to match
    if let Err(e) = send_pmtk_cmd_and_wait_ack(tx, rx, PMTK_API_SET_FIX_CTL_10HZ).await {
        // Handle the error, e.g., log it or take corrective action
        info!("Error: {}", e);
    }
}

/// Calculate NMEA checksum (XOR of all bytes between '$' and '*')
fn calculate_checksum(sentence: &str) -> u8 {
    sentence.bytes().fold(0u8, |acc, b| acc ^ b)
}

/// Sends a PMTK command and waits for an ACK response.
pub async fn send_pmtk_cmd_and_wait_ack(
    tx: &mut UartTx<'static, Async>,
    rx: &mut UartRx<'static, Async>,
    command_body: &str,
) -> Result<(), &'static str> {
    let checksum = calculate_checksum(command_body);

    let mut full_msg: String<128> = String::new();
    core::write!(full_msg, "${}*{:02X}\r\n", command_body, checksum).map_err(|_| "fmt err")?;

    // Send the command byte-by-byte
    for b in full_msg.as_bytes() {
        tx.write(&[*b]).await.map_err(|_| "TX failed")?;
    }

    info!("Sent: {}", full_msg.trim());

    let mut buf = [0u8; 1];
    let mut response: String<128> = String::new();

    let cmd_number = command_body
        .split(',')
        .next()
        .and_then(|s| s.strip_prefix("PMTK"))
        .ok_or("Invalid command")?;

    
    loop {
        // Set a timeout duration for waiting for the ACK (e.g., 1 second)
        let timeout = Timer::after_millis(1000); // Adjust this duration if needed
        
        match select(rx.read(&mut buf), timeout).await {
            Either::First(Ok(_)) => {
                let c = buf[0] as char;
                if response.push(c).is_err() {
                    return Err("ACK overflow");
                }

                if response.ends_with("\r\n") && response.contains("$PMTK001") {
                    let line = response.trim();
                    if line.contains(cmd_number) {
                        defmt::info!("Received ACK: {}", line);
                        return Ok(());
                    } else {
                        defmt::warn!("Unrelated ACK: {}", line);
                        response.clear();
                    }
                }

                if response.len() > 120 {
                    return Err("ACK overflow");
                }
            },
            Either::First(Err(_)) => return Err("RX failed"),
            Either::Second(_) => return Err("ACK timeout"),
        }
    }
}