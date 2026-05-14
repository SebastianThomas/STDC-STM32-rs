use core::fmt::Debug;
use core::option::Option::{self, None, Some};

use rtic_monotonics::Monotonic;
use rtic_monotonics::fugit::ExtU64;
use rtt_target::rprintln;
use stm32l4xx_hal::hal::blocking::spi;

use stdc_diving_algorithms::pressure_unit::{Bar, Pa, Pressure, mBar, msw};

use crate::constants::barometric::{
    ALT_PER_FOOT, DepthOrAltitude, FEET_TO_METERS, KG_M2_FRESH_WATER, RLGM,
};
use crate::protocols::spi::{spi_read_register, spi_write_delay};
use crate::stm32::Mono;

use libm::powf;

// use byteorder::{BigEndian, ByteOrder};
// use i2cdev::core::I2CDevice;
use stm32l4xx_hal::{
    hal::blocking::i2c::{Read, Write, WriteRead},
    prelude::*,
};

pub const MS5849_ADDR: u8 = 0x76;
pub const MS5849_I2C_ADDR: u8 = 0x77;
// pub const MS5849_I2C_ADDR: u8 = 0xEC;
// pub const MS5849_RESET: u8 = 0x1E;
// pub const MS5849_ADC_READ: u8 = 0x00;
// pub const MS5849_PROM_READ: u8 = 0xA0;
// pub const MS5849_CONVERT_D1_8192: u8 = 0x4A;
// pub const MS5849_CONVERT_D2_8192: u8 = 0x5A;

pub const MS5849_RESET: u8 = 0x10;
pub const MS5849_ADC_READ_D1: u8 = 0x54;
pub const MS5849_ADC_READ_D2: u8 = 0x58;
pub const MS5849_PROM_READ: u8 = 0xE0;
pub const MS5849_CONVERT_D1_8192: u8 = 0x44;
pub const MS5849_CONVERT_D2_8192: u8 = 0x48;

pub struct SensorConfiguration {
    /** g/l */
    pub fluid_density: u16,
    /** g/l * m / s^2 */
    pub fluid_density_10m: f32,
}

impl SensorConfiguration {
    pub fn new(fluid_density: u16) -> Self {
        return Self {
            fluid_density,
            fluid_density_10m: fluid_density as f32 * 9.80665,
        };
    }
}

#[allow(non_camel_case_types)]
#[allow(non_snake_case)]
/// MS5849 Pressure Sensor Driver
///
/// The MS5849 has an interrupt pin (INT/PB5) that signals when conversion is complete.
/// When a conversion is started with CONVERT_D1_8192 or CONVERT_D2_8192, the sensor
/// performs the ADC conversion and pulls the interrupt pin low when ready (~20ms max).
///
/// **Current Implementation**: Polling-based with fixed delays (20ms per conversion).
/// **Interrupt-Based Alternative**:
/// - Monitor PB5 EXTI for falling edge (conversion complete)
/// - Eliminates fixed sleep delays (~40ms total, currently)
/// - Enables more responsive dive computer, especially during rapid depth changes
/// - Requires: EXTI interrupt handler + async state machine
///
/// The interrupt signal allows you to:
/// 1. Start both D1 and D2 conversions without waiting
/// 2. Use an EXTI interrupt to receive notification when data is ready
/// 3. Reduce CPU utilization and sleep time
pub struct MS5849<INTERFACE, P> {
    interface: INTERFACE,
    config: SensorConfiguration,
    cs: Option<P>,
    P: Option<i32>,
    TEMP: Option<i32>,
    D1_pres: Option<u32>,
    D2_temp: Option<u32>,
    C: [u16; 10],
}

#[allow(unused)]
fn scan_i2c<I: Write, L: Fn(&[u8]) -> ()>(i2c: &mut I, log_bytes: L)
where
    <I as cortex_m::prelude::_embedded_hal_blocking_i2c_Write>::Error: Debug,
{
    rprintln!("Scanning I2C bus...");
    log_bytes(b"Scanning I2C bus...");

    for addr in 0x08u8..0x78u8 {
        rprintln!("Trying 0x{:02X}", addr);

        match i2c.write(addr, &[0x00]) {
            Ok(_) => {
                rprintln!("FOUND: 0x{:02X}", addr);
            }
            Err(e) => {
                rprintln!("0x{:02X}: {:?}", addr, e);
            }
        }
    }
    rprintln!("Scan complete.");
}

fn write_i2c<I: Write>(i2c: &mut I, addr: u8, data: &[u8])
where
    <I as cortex_m::prelude::_embedded_hal_blocking_i2c_Write>::Error: Debug,
{
    if let Err(e) = i2c.write(addr, data) {
        rprintln!("Write failed: {:?}", e);
        panic!("Write failed");
    }
}

fn write_read_i2c<I: WriteRead, const NR: usize>(i2c: &mut I, addr: u8, reg: u8) -> [u8; NR]
where
    <I as WriteRead>::Error: Debug,
{
    let mut result: [u8; NR] = [0; NR];
    if let Err(e) = i2c.write_read(addr, &[reg], &mut result) {
        rprintln!("Reading failed: {:?}", e);
        panic!("Reading failed");
    }
    result
}

impl<SPI, P> MS5849<SPI, P>
where
    SPI: spi::Transfer<u8> + spi::Write<u8>,
    P: OutputPin,
{
    pub async fn new_spi<L: Fn(&[u8]) -> ()>(
        mut spi: SPI,
        mut cs: P,
        log_bytes: L,
    ) -> MS5849<SPI, P> {
        let mut prom: [u16; 16] = [0; 16];

        rprintln!("Created SPI Interface");
        log_bytes(b"Created SPI Interface");

        // let mut cs_pins = [&mut cs];
        // scan_spi(&mut spi, &mut cs_pins, 0xA0);

        /* Reset and Calibrate */
        // Reset the MS5837, per datasheet
        spi_write_delay(&mut spi, &mut cs, MS5849_RESET, 300).await;
        rprintln!("Reset SPI");

        // Read full PROM map (words 0..15).
        for i in 0..=15 {
            let mut buf: [u8; 3] = [MS5849_PROM_READ + i * 2, 0, 0];
            spi_read_register(&mut spi, &mut cs, &mut buf);
            prom[i as usize] = (buf[1] as u16) << 8 | (buf[2] as u16);
        }

        let mut instance = Self::build_from_prom(spi, Some(cs), prom);
        instance.read_spi().await;
        instance
    }

    pub async fn read_spi(&mut self) {
        // Request D1 conversion
        spi_write_delay(
            &mut self.interface,
            self.cs.as_mut().expect("SPI requires CS"),
            MS5849_CONVERT_D1_8192,
            20,
        )
        .await;

        let mut buf: [u8; 4] = [MS5849_ADC_READ_D1, 0, 0, 0];
        spi_read_register(&mut self.interface, self.cs.as_mut().unwrap(), &mut buf);
        let d1_pres = (buf[1] as u32) << 16 | (buf[2] as u32) << 8 | buf[3] as u32;
        self.D1_pres = Some(d1_pres);

        // Request D2 conversion
        spi_write_delay(
            &mut self.interface,
            self.cs.as_mut().unwrap(),
            MS5849_CONVERT_D2_8192,
            20,
        )
        .await;

        let mut buf: [u8; 4] = [MS5849_ADC_READ_D2, 0, 0, 0];
        spi_read_register(&mut self.interface, self.cs.as_mut().unwrap(), &mut buf);
        let d2_temp = (buf[1] as u32) << 16 | (buf[2] as u32) << 8 | buf[3] as u32;
        self.update_raw_measurements(d1_pres, d2_temp);
    }
}

impl<I: Write + Read + WriteRead> MS5849<I, ()>
where
    <I as Read>::Error: Debug,
    <I as Write>::Error: Debug,
    <I as WriteRead>::Error: Debug,
{
    pub async fn new_i2c<L: Fn(&[u8]) -> ()>(mut i2c: I, log_bytes: L) -> MS5849<I, ()> {
        let mut prom: [u16; 16] = [0; 16];

        // Reset the MS5849, per datasheet
        write_i2c(&mut i2c, MS5849_I2C_ADDR, &[MS5849_RESET]);

        rprintln!("Reset MS5849 Pressure Sensor, waiting to get ready");
        log_bytes(b"Reset MS5849 Pressure Sensor, waiting to get ready");

        Mono::delay(300u64.micros()).await;

        // Read full PROM map (words 0..15).
        for i in 0..=15 {
            let buf: [u8; 2] =
                write_read_i2c(&mut i2c, MS5849_I2C_ADDR, MS5849_PROM_READ + (i << 1));
            prom[i as usize] = (buf[0] as u16) << 8 | buf[1] as u16;
        }

        let mut instance = Self::build_from_prom(i2c, None, prom);
        instance.read_i2c().await;
        instance
    }

    pub async fn read_i2c(&mut self) {
        // Request D1 conversion
        rprintln!("MS5849 read_i2c: request D1 conversion");
        write_i2c(
            &mut self.interface,
            MS5849_I2C_ADDR,
            &[MS5849_CONVERT_D1_8192],
        );

        // Request D2 conversion
        rprintln!("MS5849 read_i2c: request D2 conversion");
        write_i2c(
            &mut self.interface,
            MS5849_I2C_ADDR,
            &[MS5849_CONVERT_D2_8192],
        );

        rprintln!("MS5849 read_i2c: waiting on Mono for 20ms");

        Mono::delay(20u64.millis()).await; // Max conversion time per datasheet

        rprintln!("MS5849 read_i2c: Mono delay completed");

        let buf: [u8; 3] = write_read_i2c(&mut self.interface, MS5849_I2C_ADDR, MS5849_ADC_READ_D1);
        let d1_pres = (buf[0] as u32) << 16 | (buf[1] as u32) << 8 | buf[2] as u32;
        self.D1_pres = Some(d1_pres);

        let buf: [u8; 3] = write_read_i2c(&mut self.interface, MS5849_I2C_ADDR, MS5849_ADC_READ_D2);
        let d2_temp = (buf[0] as u32) << 16 | (buf[1] as u32) << 8 | buf[2] as u32;
        self.update_raw_measurements(d1_pres, d2_temp);
    }
}

impl<I, P> MS5849<I, P> {
    fn build_from_prom(interface: I, cs: Option<P>, prom: [u16; 16]) -> MS5849<I, P> {
        let serial_number: [u16; 2] = [prom[2], prom[3]];
        let cal = Self::calibration_from_prom(&prom);

        rprintln!(
            "Serial Number of MS5849 Pressure Sensor: 0x{:04x}{:04x}",
            serial_number[0],
            serial_number[1]
        );
        rprintln!("Got calibration values: {:?}", cal);

        Self::validate_prom_or_panic(&prom);

        let config = SensorConfiguration::new(KG_M2_FRESH_WATER);
        MS5849 {
            interface,
            config,
            cs,
            P: None,
            TEMP: None,
            D1_pres: None,
            D2_temp: None,
            C: cal,
        }
    }

    fn calibration_from_prom(prom: &[u16; 16]) -> [u16; 10] {
        let mut cal: [u16; 10] = [0; 10];
        for (idx, coeff) in cal.iter_mut().enumerate() {
            *coeff = prom[idx + 4];
        }
        cal
    }

    fn validate_prom_or_panic(prom: &[u16; 16]) {
        let product_id = (prom[15] >> 8) as u8;
        let crc_read = prom[15] as u8;

        if product_id != 0x01 {
            rprintln!(
                "Product ID of Pressure Sensor should be 0x01, is: {:02x}",
                product_id
            );
            panic!("Pressure Sensor broken or not compatible.");
        }

        let crc_calculated = Self::prom_crc8(prom);
        if crc_calculated != crc_read {
            rprintln!(
                "CRC8 failed: read=0x{:02X}, calculated=0x{:02X}",
                crc_read,
                crc_calculated
            );
            panic!("Calibration CRC8 verification failed");
        }
        rprintln!("CRC8 validation passed (0x{:02X})", crc_read);
    }

    fn update_raw_measurements(&mut self, d1_pres: u32, d2_temp: u32) {
        self.D1_pres = Some(d1_pres);
        self.D2_temp = Some(d2_temp);
        self.calculate();
    }

    pub fn current_pressure_bar(&self) -> Option<Bar> {
        self.pressure().map(|pa: Pa| pa.to_bar())
    }

    pub fn current_pressure_pa(&self) -> Option<Pa> {
        self.pressure()
    }

    #[allow(non_snake_case)]
    pub fn calculate(&mut self) -> Option<()> {
        // Raw values
        let d1_pres = self.D1_pres? as f32;
        let d2_temp = self.D2_temp? as f32;

        let temperature = (self.C[0] as f32 * d2_temp) / (1 << 29) as f32
            - (self.C[2] as f32 * d1_pres) / (1u64 << 35) as f32
            - self.C[1] as f32 / (1 << 6) as f32;

        let OFF = self.C[5] as f32 + self.C[6] as f32 * temperature / (1 << 9) as f32;
        let SENS = self.C[7] as f32 + self.C[8] as f32 * temperature / (1 << 9) as f32;
        let pressure = d1_pres * SENS / ((1 << 22) as f32) - OFF;

        self.P = Some((pressure * 10.0) as i32);
        self.TEMP = Some((temperature * 100.0) as i32);

        // TODO: Second order compensation:
        // https://download.mikroe.com/documents/datasheets/MS5849-30BA_datasheet.pdf page 11
        Some(())
    }

    pub fn pressure(&self) -> Option<Pa> {
        return self.P.map(|p| Pa::new(p as f32 * 10.0));
    }

    pub fn temperature(&self) -> Option<f32> {
        return self.TEMP.map(|t| t as f32 / 100.0);
    }

    pub fn depth_relative_or_altitude(
        &self,
        surf_ref: Pa,
    ) -> Result<DepthOrAltitude, (Option<Pa>, &str)> {
        let pressure = match self.pressure() {
            Some(p) => p,
            None => return Err((None, "No pressure")),
        };
        // TODO: Allow slight difference
        const SUBMERGED_DIFF: Pa = mBar::new(50.0).to_pa(); // half a meter
        let is_submerged = pressure - surf_ref > SUBMERGED_DIFF;
        if is_submerged {
            rprintln!("Is submerged");
            let pressure_below = pressure - surf_ref;
            let depth: f32 = (pressure_below / self.config.fluid_density_10m).to_f32();
            return Ok(DepthOrAltitude::Depth {
                pressure,
                depth: msw(depth),
            });
        } else {
            let p = pressure / surf_ref;
            let ratio = 1.0 - powf(p, RLGM);
            return Ok(DepthOrAltitude::Altitude {
                pressure,
                altitude: ratio * ALT_PER_FOOT * FEET_TO_METERS,
            });
        }
    }
}

impl<I, P> MS5849<I, P> {
    fn crc8_update(data_in: u8, crc_init: u8) -> u8 {
        // Same CRC8 update as the C reference driver (poly 0x31).
        let mut u_dat = data_in;
        let mut u_rem = crc_init;

        for _ in 0..8 {
            let msb_dat = u_dat >> 7;
            let msb_rem = u_rem >> 7;
            u_dat <<= 1;
            u_rem <<= 1;
            if msb_dat != msb_rem {
                u_rem ^= 0x31;
            }
        }

        u_rem
    }

    /// Calculate CRC8 over full PROM data:
    /// words[0..14] (all bytes) + word[15] high byte (product_id),
    /// excluding the stored CRC byte in word[15] low byte.
    fn prom_crc8(prom: &[u16; 16]) -> u8 {
        let mut crc: u8 = 0;

        for &word in prom.iter().take(15) {
            crc = Self::crc8_update((word >> 8) as u8, crc);
            crc = Self::crc8_update(word as u8, crc);
        }

        Self::crc8_update((prom[15] >> 8) as u8, crc)
    }
}
