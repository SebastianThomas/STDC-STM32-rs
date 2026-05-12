use core::fmt::Debug;
use core::option::Option::{self, None, Some};

use core::cell::RefCell;
use cortex_m::interrupt::{Mutex, free};
use rtt_target::rprintln;
use stm32l4xx_hal::hal::blocking::spi;

use stdc_diving_algorithms::pressure_unit::{Bar, Pa, Pressure, mBar, msw};

use crate::constants::barometric::{
    ALT_PER_FOOT, DepthOrAltitude, FEET_TO_METERS, KG_M2_FRESH_WATER, RLGM, SURFACE_PA,
};
use crate::protocols::spi::{spi_read_register, spi_write_delay};

use libm::powf;

// use byteorder::{BigEndian, ByteOrder};
// use i2cdev::core::I2CDevice;
use stm32l4xx_hal::{
    delay::Delay,
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
pub struct MS5849<'a, INTERFACE, P> {
    interface: INTERFACE,
    config: SensorConfiguration,
    cs: Option<P>,
    P: Option<i32>,
    TEMP: Option<i32>,
    D1_pres: Option<u32>,
    D2_temp: Option<u32>,
    C: [u16; 10],
    delay: &'a Mutex<RefCell<Delay>>,
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

fn wait_us(delay: &Mutex<RefCell<Delay>>, us: u16) {
    free(|cs| delay.borrow(cs).borrow_mut().delay_ms(us));
}

fn wait_ms(delay: &Mutex<RefCell<Delay>>, ms: u8) {
    free(|cs| delay.borrow(cs).borrow_mut().delay_ms(ms));
}

impl<'a, SPI, P> MS5849<'a, SPI, P>
where
    SPI: spi::Transfer<u8> + spi::Write<u8>,
    P: OutputPin,
{
    pub fn new_spi<L: Fn(&[u8]) -> ()>(
        mut spi: SPI,
        mut cs: P,
        delay: &'a Mutex<RefCell<Delay>>,
        log_bytes: L,
    ) -> MS5849<'a, SPI, P> {
        let mut cal: [u16; 10] = [0; 10];

        rprintln!("Created SPI Interface");
        log_bytes(b"Created SPI Interface");

        // let mut cs_pins = [&mut cs];
        // scan_spi(&mut spi, &mut cs_pins, 0xA0);

        /* Reset and Calibrate */
        // Reset the MS5837, per datasheet
        spi_write_delay(&mut spi, &mut cs, MS5849_RESET, || wait_us(delay, 300_u16));
        rprintln!("Reset SPI");

        let mut serial_number: [u16; 2] = [0; 2];
        for i in 2..=3 {
            let mut buf: [u8; 3] = [MS5849_PROM_READ + i * 2, 0, 0];
            spi_read_register(&mut spi, &mut cs, &mut buf);
            serial_number[(i - 2) as usize] = (buf[1] as u16) << 8 | (buf[2] as u16);
            rprintln!("Serial number {:?} result: {:?}", i, buf);
        }
        rprintln!("Got serial number: {:?}", serial_number);

        // Read calibration values and CRC
        for i in 4..=13 {
            let cmd = MS5849_PROM_READ + i * 2;
            rprintln!("Reading from 0x{:02X}", cmd);
            let mut buf: [u8; 3] = [cmd, 0, 0];
            spi_read_register(&mut spi, &mut cs, &mut buf);
            rprintln!("Got result: {:?}", buf);
            cal[(i - 4) as usize] = (buf[1] as u16) << 8 | buf[2] as u16;
        }

        rprintln!("Calibration data: {:?}", cal);

        // TODO: Verify that data is correct with CRC
        // let crc_read: u8 = (cal[0] >> 12) as u8;
        // let crc_calculated: u8 = crc4(&mut cal);

        // if crc_calculated != crc_read {
        //     // CRC fail
        //     rprintln!("CRC failed");
        //     panic!("CRC failed");
        // }

        let config = SensorConfiguration::new(KG_M2_FRESH_WATER);
        MS5849 {
            interface: spi,
            config,
            cs: Some(cs),
            P: None,
            TEMP: None,
            D1_pres: None,
            D2_temp: None,
            C: cal,
            delay,
        }
    }

    pub fn read_spi(&mut self) {
        // Request D1 conversion
        spi_write_delay(
            &mut self.interface,
            self.cs.as_mut().expect("SPI requires CS"),
            MS5849_CONVERT_D1_8192,
            || wait_ms(&self.delay, 20_u8),
        ); // Max conversion time per datasheet

        let mut buf: [u8; 4] = [MS5849_ADC_READ_D1, 0, 0, 0];
        spi_read_register(&mut self.interface, self.cs.as_mut().unwrap(), &mut buf);
        let d1_pres = (buf[1] as u32) << 16 | (buf[2] as u32) << 8 | buf[3] as u32;
        self.D1_pres = Some(d1_pres);

        // Request D2 conversion
        spi_write_delay(
            &mut self.interface,
            self.cs.as_mut().unwrap(),
            MS5849_CONVERT_D2_8192,
            || wait_ms(&self.delay, 20_u8),
        ); // Max conversion time per datasheet

        let mut buf: [u8; 4] = [MS5849_ADC_READ_D2, 0, 0, 0];
        spi_read_register(&mut self.interface, self.cs.as_mut().unwrap(), &mut buf);
        let d2_temp = (buf[1] as u32) << 16 | (buf[2] as u32) << 8 | buf[3] as u32;
        self.D2_temp = Some(d2_temp);

        self.calculate();
    }
}

impl<'a, I: Write + Read + WriteRead> MS5849<'a, I, ()>
where
    <I as Read>::Error: Debug,
    <I as Write>::Error: Debug,
    <I as WriteRead>::Error: Debug,
{
    pub fn new_i2c<L: Fn(&[u8]) -> ()>(
        mut i2c: I,
        delay: &'a Mutex<RefCell<Delay>>,
        log_bytes: L,
    ) -> MS5849<'a, I, ()> {
        let mut cal: [u16; 10] = [0; 10];

        // Reset the MS5849, per datasheet
        write_i2c(&mut i2c, MS5849_I2C_ADDR, &[MS5849_RESET]);

        rprintln!("Reset MS5849 Pressure Sensor, waiting to get ready");
        log_bytes(b"Reset MS5849 Pressure Sensor, waiting to get ready");

        wait_us(delay, 300_u16);

        let serial_nr_msbs: [u8; 2] =
            write_read_i2c(&mut i2c, MS5849_I2C_ADDR, MS5849_PROM_READ + (2 << 1));
        let serial_nr_lsbs: [u8; 2] =
            write_read_i2c(&mut i2c, MS5849_I2C_ADDR, MS5849_PROM_READ + (3 << 1));
        rprintln!(
            "Serial Number of MS5849 Pressure Sensor: 0x{:02x}{:02x}{:02x}{:02x}",
            serial_nr_msbs[0],
            serial_nr_msbs[1],
            serial_nr_lsbs[0],
            serial_nr_lsbs[1],
        );

        // Read calibration values and CRC
        for i in 4..=13 {
            let buf: [u8; 2] =
                write_read_i2c(&mut i2c, MS5849_I2C_ADDR, MS5849_PROM_READ + (i << 1));
            cal[(i - 4) as usize] = (buf[0] as u16) << 8 | buf[1] as u16;
        }
        rprintln!("Got calibration values: {:?}", cal);

        let product_id_and_crc: [u8; 2] =
            write_read_i2c(&mut i2c, MS5849_I2C_ADDR, MS5849_PROM_READ + (15 << 1));

        if product_id_and_crc[0] != 0x01 {
            rprintln!(
                "Product ID of Pressure Sensor should be 0x01, is: {:02x}",
                product_id_and_crc[0]
            );
            panic!("Pressure Sensor broken or not compatible.");
        }

        // TODO: Verify that data is correct with CRC
        // let crc_read: u8 = (cal[0] >> 12) as u8;
        // let crc_calculated: u8 = crc4(&mut cal);

        // if crc_calculated != crc_read {
        //     // CRC fail
        //     rprintln!("CRC failed");
        //     panic!("CRC failed");
        // }

        let config = SensorConfiguration::new(KG_M2_FRESH_WATER);
        MS5849 {
            interface: i2c,
            config,
            cs: None,
            P: None,
            TEMP: None,
            D1_pres: None,
            D2_temp: None,
            C: cal,
            delay,
        }
    }

    pub fn read_i2c(&mut self) {
        // Request D1 conversion
        write_i2c(
            &mut self.interface,
            MS5849_I2C_ADDR,
            &[MS5849_CONVERT_D1_8192],
        );

        self.wait_ms(20); // Max conversion time per datasheet

        let buf: [u8; 3] = write_read_i2c(&mut self.interface, MS5849_I2C_ADDR, MS5849_ADC_READ_D1);
        let d1_pres = (buf[0] as u32) << 16 | (buf[0] as u32) << 8 | buf[0] as u32;
        self.D1_pres = Some(d1_pres);

        // Request D2 conversion
        write_i2c(
            &mut self.interface,
            MS5849_I2C_ADDR,
            &[MS5849_CONVERT_D2_8192],
        );

        self.wait_ms(20); // Max conversion time per datasheet

        let buf: [u8; 3] = write_read_i2c(&mut self.interface, MS5849_I2C_ADDR, MS5849_ADC_READ_D2);
        let d2_temp = (buf[0] as u32) << 16 | (buf[0] as u32) << 8 | buf[0] as u32;
        self.D2_temp = Some(d2_temp);

        self.calculate();
    }
}

impl<'a, I, P> MS5849<'a, I, P> {
    pub fn current_pressure_bar(&self) -> Option<Bar> {
        self.pressure().map(|pa: Pa| pa.to_bar())
    }

    pub fn current_pressure_pa(&self) -> Option<Pa> {
        self.pressure()
    }

    fn wait_ms(&self, ms: u8) {
        wait_ms(&self.delay, ms);
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
            let p = pressure / SURFACE_PA;
            let ratio = 1.0 - powf(p, RLGM);
            return Ok(DepthOrAltitude::Altitude {
                pressure,
                altitude: ratio * ALT_PER_FOOT * FEET_TO_METERS,
            });
        }
    }
}

// fn crc4(n_prom: &mut [u16; 8]) -> u8 {
//     let mut n_rem = 0 as u16;

//     n_prom[0] = (n_prom[0]) & 0x0FFF;
//     n_prom[7] = 0;

//     for i in 0..16 {
//         if i % 2 == 1 {
//             n_rem ^= ((n_prom[i >> 1]) & 0x00FF) as u16;
//         } else {
//             n_rem ^= (n_prom[i >> 1] >> 8) as u16;
//         }
//         for _n_bit in (1..=8).rev() {
//             if n_rem & 0x8000 != 0 {
//                 n_rem = (n_rem << 1) ^ 0x3000;
//             } else {
//                 n_rem = n_rem << 1;
//             }
//         }
//     }
//     n_rem = (n_rem >> 12) & 0x000F;
//     return (n_rem ^ 0x00).try_into().unwrap();
// }
