use core::fmt::Debug;
use core::option::Option::{self, None, Some};

use rtic_monotonics::Monotonic;
use rtic_monotonics::fugit::ExtU64;
use rtt_target::rprintln;
use stm32l4xx_hal::hal::blocking::spi;

use stdc_diving_algorithms::pressure_unit::{Bar, Pa, Pressure, mBar, msw};

#[cfg(feature = "live_sim")]
use crate::algorithms::profile_emulation::EmulatedDiveProfile;
use crate::constants::barometric::{
    ALT_PER_FOOT, DepthOrAltitude, FEET_TO_METERS, KG_M2_FRESH_WATER, RLGM, SURFACE_PA,
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

use crate::algorithms::profile_emulation::EmulationDecoOverlay;

#[cfg(feature = "live_sim")]
const DECO_TRANSIT_ASCENT_RATE_M_PER_MIN: f32 = 9.0;
#[cfg(feature = "live_sim")]
const LIVE_SIM_SAMPLE_INTERVAL_MS: u32 = 200;

pub trait LiveSimEmulationControl {
    fn enter_live_sim(&mut self, _surface_pressure: Pa) {}

    fn exit_live_sim(&mut self) {}

    fn sync_live_sim_dive_overlay(&mut self, _overlay: EmulationDecoOverlay) {}
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
/// 1. Use an EXTI interrupt to receive notification when data is ready
/// 2. Reduce CPU utilization and sleep time
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
    pub async fn new_i2c(mut i2c: I) -> MS5849<I, ()> {
        let mut prom: [u16; 16] = [0; 16];

        // Reset the MS5849, per datasheet
        write_i2c(&mut i2c, MS5849_I2C_ADDR, &[MS5849_RESET]);

        rprintln!("Reset MS5849 Pressure Sensor, waiting to get ready");

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
        write_i2c(
            &mut self.interface,
            MS5849_I2C_ADDR,
            &[MS5849_CONVERT_D1_8192],
        );

        write_i2c(
            &mut self.interface,
            MS5849_I2C_ADDR,
            &[MS5849_CONVERT_D2_8192],
        );

        Mono::delay(20u64.millis()).await; // Max conversion time per datasheet

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
        pressure: Option<Pa>,
        surf_ref: Pa,
    ) -> Result<DepthOrAltitude, (Option<Pa>, &str)> {
        let pressure = match pressure {
            Some(p) => p,
            None => return Err((None, "No pressure")),
        };
        const SUBMERGED_DIFF: Pa = mBar::new(50.0).to_pa(); // half a meter
        let is_submerged = pressure - surf_ref > SUBMERGED_DIFF;
        if is_submerged {
            let pressure_below = pressure - surf_ref;
            let depth: f32 = (pressure_below / self.config.fluid_density_10m).to_f32();
            return Ok(DepthOrAltitude::Depth {
                pressure,
                depth: msw(depth),
            });
        } else {
            let p = pressure / SURFACE_PA; // Not surf_ref - actual absolute altitude
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

#[cfg(feature = "live_sim")]
pub struct LiveSimMS5849<INTERFACE, P> {
    sensor: MS5849<INTERFACE, P>,
    surf_ref: Pa,
    emulated_profile: EmulatedDiveProfile,
    emulation_active: bool,
    emulation_sample_index: usize,
    emulation_last_tick_ms: u32,
}

#[cfg(feature = "live_sim")]
impl<I: Write + Read + WriteRead> LiveSimMS5849<I, ()>
where
    <I as Read>::Error: Debug,
    <I as Write>::Error: Debug,
    <I as WriteRead>::Error: Debug,
{
    pub async fn new_i2c(i2c: I) -> Self {
        let sensor = MS5849::new_i2c(i2c).await;
        let surf_ref = sensor
            .pressure()
            .expect("Only for benchmarking, should always be initialized");
        Self {
            sensor,
            surf_ref,
            emulated_profile: EmulatedDiveProfile::deep_with_defaults(LIVE_SIM_SAMPLE_INTERVAL_MS),
            emulation_active: false,
            emulation_sample_index: 0,
            emulation_last_tick_ms: Mono::now().ticks() as u32,
        }
    }

    pub async fn read_i2c(&mut self) {
        self.sensor.read_i2c().await;
    }

    pub fn current_pressure_pa(&self) -> Option<Pa> {
        self.sensor.current_pressure_pa()
    }

    pub fn temperature(&self) -> Option<f32> {
        self.sensor.temperature()
    }

    pub fn depth_relative_or_altitude(
        &mut self,
        pressure: Option<Pa>,
        surf_ref: Pa,
    ) -> Result<DepthOrAltitude, (Option<Pa>, &str)> {
        self.sensor.depth_relative_or_altitude(pressure, surf_ref)
    }

    pub fn pressure(&mut self) -> Option<Pa> {
        if !self.emulation_active {
            return self.sensor.pressure();
        }
        let now_ms = Mono::now().ticks() as u32;
        let elapsed_ms = now_ms.wrapping_sub(self.emulation_last_tick_ms);
        let advance_samples = elapsed_ms / LIVE_SIM_SAMPLE_INTERVAL_MS;
        if advance_samples > 0 {
            self.emulation_sample_index = self
                .emulation_sample_index
                .wrapping_add(advance_samples as usize);
            self.emulation_last_tick_ms = self
                .emulation_last_tick_ms
                .wrapping_add(advance_samples.saturating_mul(LIVE_SIM_SAMPLE_INTERVAL_MS));
        }
        let emulated_point = self.emulated_profile.point_at(
            self.surf_ref,
            LIVE_SIM_SAMPLE_INTERVAL_MS,
            self.emulation_sample_index,
        );
        Some(emulated_point.depth_pa)
    }
}

#[cfg(feature = "live_sim")]
impl<I, P> LiveSimEmulationControl for LiveSimMS5849<I, P> {
    fn enter_live_sim(&mut self, _surface_pressure: Pa) {
        self.surf_ref = _surface_pressure;
        #[cfg(feature = "live_sim_50m")]
        {
            use crate::algorithms::profile_emulation::EmulatedDiveProfile;

            self.emulated_profile =
                EmulatedDiveProfile::mid_50m_with_deco_defaults(LIVE_SIM_SAMPLE_INTERVAL_MS);
        }
        #[cfg(feature = "live_sim_20m")]
        {
            self.emulated_profile =
                EmulatedDiveProfile::shallow_20m_with_defaults(LIVE_SIM_SAMPLE_INTERVAL_MS);
        }
        #[cfg(feature = "live_sim_90m")]
        {
            self.emulated_profile =
                EmulatedDiveProfile::deep_with_defaults(LIVE_SIM_SAMPLE_INTERVAL_MS);
        }

        self.emulation_active = true;
        self.emulation_sample_index = 0;
        self.emulation_last_tick_ms = Mono::now().ticks() as u32;
    }

    fn exit_live_sim(&mut self) {
        self.emulation_active = false;
        self.emulation_sample_index = 0;
        self.emulation_last_tick_ms = Mono::now().ticks() as u32;
    }

    fn sync_live_sim_dive_overlay(&mut self, overlay: EmulationDecoOverlay) {
        if overlay.count == 0 {
            self.emulated_profile = self.emulated_profile.without_deco_overlay();
        } else {
            let mut absolute_overlay = overlay;
            let current_point = self.emulated_profile.point_at(
                self.surf_ref,
                LIVE_SIM_SAMPLE_INTERVAL_MS,
                self.emulation_sample_index,
            );
            let mut previous_depth = current_point.depth_msw.to_pa();
            let mut current_start = self.emulation_sample_index;

            for stop in absolute_overlay
                .stops
                .iter_mut()
                .take(absolute_overlay.count)
            {
                let previous_depth_m = previous_depth.to_msw().to_f32();
                let stop_depth_m = stop.stop_depth_pa.to_msw().to_f32();
                let transit_depth_m = (previous_depth_m - stop_depth_m).max(0.0);
                let transit_samples = if transit_depth_m > 0.0 {
                    let transit_secs =
                        (transit_depth_m / DECO_TRANSIT_ASCENT_RATE_M_PER_MIN) * 60.0;
                    libm::ceilf((transit_secs * 1000.0) / LIVE_SIM_SAMPLE_INTERVAL_MS as f32)
                        as usize
                } else {
                    0
                };

                current_start = current_start.saturating_add(transit_samples);
                stop.start_sample = Some(current_start);
                current_start = current_start.saturating_add(stop.hold_samples);
                previous_depth = stop.stop_depth_pa;
            }

            self.emulated_profile = self
                .emulated_profile
                .with_deco_overlay_obj(absolute_overlay);
        }
    }
}
