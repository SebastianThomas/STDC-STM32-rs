pub mod barometric {
    use thalmann::pressure_unit::{Bar, Pa, Pressure, hPa, kPa};

    pub const SURFACE_PA: Pa = Pa::new(101325.0);
    pub const SURFACE_HPA: hPa = SURFACE_PA.to_hpa();
    pub const SURFACE_KPA: kPa = SURFACE_PA.to_kpa();
    pub const SURFACE_BAR: Bar = SURFACE_PA.to_bar();

    pub const FEET_TO_METERS: f32 = 0.3048;
    pub const ALT_PER_FOOT: f32 = 145366.45;
}

pub mod ms5849_pressure {
    use core::fmt::Debug;
    use core::option::Option::{self, None, Some};

    use core::cell::RefCell;
    use cortex_m::interrupt::{Mutex, free};
    use rtt_target::rprintln;
    use stm32l4xx_hal::hal::blocking::spi;

    use thalmann::pressure_unit::{Bar, Pa, Pressure};

    use crate::spi::{spi_read_register, spi_write_delay};

    use libm::powf;

    // use byteorder::{BigEndian, ByteOrder};
    // use i2cdev::core::I2CDevice;
    use stm32l4xx_hal::{
        delay::Delay,
        hal::blocking::i2c::{Read, Write, WriteRead},
        prelude::*,
    };

    use crate::ms5849::barometric::*;

    pub const MS5849_ADDR: u8 = 0x76;
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

    #[allow(non_camel_case_types)]
    #[allow(non_snake_case)]
    pub struct MS5849<'a, INTERFACE, P> {
        /** g/l */
        fluid_density: u16,
        interface: INTERFACE,
        cs: Option<P>,
        P: Option<i32>,
        TEMP: Option<i32>,
        D1_pres: Option<u32>,
        D2_temp: Option<u32>,
        C: [u16; 10],
        delay: &'a Mutex<RefCell<Delay>>,
    }

    fn scan_i2c<I: Write>(i2c: &mut I)
    where
        <I as cortex_m::prelude::_embedded_hal_blocking_i2c_Write>::Error: Debug,
    {
        rprintln!("Scanning I2C bus...");

        for addr in 0x03..0x78 {
            // valid 7-bit I2C addresses
            // Try a zero-length write (common method to ping device)
            let result = i2c.write(addr, &[]);
            match result {
                Ok(_) => rprintln!("I2C device found at 0x{:02X}", addr),
                // Err(res) => rprintln!("No I2C device found at 0x{:02X}: {:?}", addr, res),
                Err(_) => (),
            }
        }
        rprintln!("Scan complete.");
    }

    fn write_i2c<I: Write>(i2c: &mut I, addr: u8, data: &[u8])
    where
        <I as cortex_m::prelude::_embedded_hal_blocking_i2c_Write>::Error: Debug,
    {
        rprintln!("Writing to {:#x}: {:?}", addr, data);
        if let Err(e) = i2c.write(addr, data) {
            rprintln!("Write failed: {:?}", e);
            panic!("Write failed");
        }
        rprintln!("Successfully written to {:#x}: {:?}", addr, data);
    }

    fn read_i2c<I: Read, const NR: usize>(i2c: &mut I, addr: u8) -> [u8; NR]
    where
        <I as cortex_m::prelude::_embedded_hal_blocking_i2c_Read>::Error: Debug,
    {
        let mut result: [u8; NR] = [0; NR];
        if let Err(e) = i2c.read(addr, &mut result) {
            rprintln!("Reading failed: {:?}", e);
            panic!("Reading failed");
        }
        result
    }

    fn wait_us(delay: &Mutex<RefCell<Delay>>, us: u16) {
        free(|cs| delay.borrow(cs).borrow_mut().delay_ms(us)); // Max conversion time per datasheet
    }

    fn wait_ms(delay: &Mutex<RefCell<Delay>>, ms: u8) {
        free(|cs| delay.borrow(cs).borrow_mut().delay_ms(ms)); // Max conversion time per datasheet
    }

    impl<'a, SPI, P> MS5849<'a, SPI, P>
    where
        SPI: spi::Transfer<u8> + spi::Write<u8>,
        P: OutputPin,
    {
        pub fn new_spi(
            mut spi: SPI,
            mut cs: P,
            delay: &'a Mutex<RefCell<Delay>>,
        ) -> MS5849<'a, SPI, P> {
            let mut cal: [u16; 10] = [0; 10];

            rprintln!("Created SPI Interface");

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

            MS5849 {
                fluid_density: 1029,
                interface: spi,
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
        pub fn new_i2c(mut i2c: I, delay: &'a Mutex<RefCell<Delay>>) -> MS5849<'a, I, ()> {
            let mut cal: [u16; 10] = [0; 10];

            rprintln!("Created I2C Interface");

            scan_i2c(&mut i2c);

            /* Reset and Calibrate */
            // Reset the MS5837, per datasheet
            write_i2c(&mut i2c, MS5849_ADDR, &[MS5849_RESET]);
            rprintln!("Written for the first time");

            // Read calibration values and CRC
            for i in 0..7 {
                // TODO: New MS5849 with new addresses from 4-13
                write_i2c(&mut i2c, MS5849_ADDR, &[MS5849_PROM_READ + i * 2]);

                // let mut buf: [u8; 2] = [0; 2];
                // if let Err(e) = i2c.read(MS5849_ADDR, &mut buf) {
                //     rprintln!("Converting failed: {:?}", e);
                //     panic!("Converting failed");
                // }
                let buf: [u8; 2] = read_i2c(&mut i2c, MS5849_ADDR);
                cal[i as usize] = (buf[0] as u16) << 8 | buf[1] as u16;
            }

            // TODO: Verify that data is correct with CRC
            // let crc_read: u8 = (cal[0] >> 12) as u8;
            // let crc_calculated: u8 = crc4(&mut cal);

            // if crc_calculated != crc_read {
            //     // CRC fail
            //     rprintln!("CRC failed");
            //     panic!("CRC failed");
            // }

            MS5849 {
                fluid_density: 1029,
                interface: i2c,
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
            write_i2c(&mut self.interface, MS5849_ADDR, &[MS5849_CONVERT_D1_8192]);

            self.wait_ms(20u8); // Max conversion time per datasheet

            write_i2c(&mut self.interface, MS5849_ADDR, &[MS5849_ADC_READ_D1]);

            let buf: [u8; 3] = read_i2c(&mut self.interface, MS5849_ADDR);
            let d1_pres = (buf[0] as u32) << 16 | (buf[0] as u32) << 8 | buf[0] as u32;
            self.D1_pres = Some(d1_pres);

            // Request D2 conversion
            write_i2c(&mut self.interface, MS5849_ADDR, &[MS5849_CONVERT_D2_8192]);

            self.wait_ms(20u8); // Max conversion time per datasheet

            write_i2c(&mut self.interface, MS5849_ADDR, &[MS5849_ADC_READ_D2]);

            let buf: [u8; 3] = read_i2c(&mut self.interface, MS5849_ADDR);
            let d2_temp = (buf[0] as u32) << 16 | (buf[0] as u32) << 8 | buf[0] as u32;
            self.D2_temp = Some(d2_temp);

            self.calculate();
        }
    }

    impl<'a, I, P> MS5849<'a, I, P> {
        pub fn measure_pressure_to_bar(&self) -> Option<Bar> {
            self.pressure().map(|pa: Pa| pa.to_bar())
        }

        fn wait_ms(&self, ms: u8) {
            wait_ms(&self.delay, ms);
        }
        #[allow(non_snake_case)]
        pub fn calculate(&mut self) -> Option<()> {
            // Given C1-C6 and D1, D2, calculated TEMP and P
            // Do conversion first and then second order temp compensation

            let mut SENSi: i64 = 0;
            let mut OFFi: i64 = 0;
            let mut Ti: i64 = 0;
            let OFF2: i64;
            let SENS2: i64;

            let d1_pres = self.D1_pres? as i64;
            let d2_temp = self.D2_temp? as i64;

            // Terms called
            let dT: i64 = d2_temp as i64 - (self.C[5] as i64) * 256;
            let SENS: i64 = (self.C[1] as i64) * 32768 + ((self.C[3] as i64) * dT) / 256;
            let OFF: i64 = (self.C[2] as i64) * 65536 + ((self.C[4] as i64) * dT) / 128;
            self.P = Some(((d1_pres * SENS / (2097152 as i64) - OFF) / (8192)) as i32);

            // Temp conversion
            let temperature = 2000 + (dT) * self.C[6] as i64 / 8388608 as i64;

            //Second order compensation
            if (temperature / 100) < 20 {
                //Low temp
                Ti = (3 * dT * dT) / (8589934592 as i64);
                OFFi = (3 * (temperature - 2000) * (temperature - 2000)) / 2;
                SENSi = (5 * (temperature - 2000) * (temperature - 2000)) / 8;
                if (temperature / 100) < -15 {
                    //Very low temp
                    OFFi = OFFi as i64 + 7 * (temperature + 1500) * (temperature + 1500);
                    SENSi = SENSi as i64 + 4 * (temperature + 1500) * (temperature + 1500);
                }
            } else if (temperature / 100) >= 20 {
                //High temp
                Ti = 2 * (dT * dT) / (137438953472 as i64);
                OFFi = (1 * (temperature - 2000) * (temperature - 2000)) / 16;
                SENSi = 0;
            }

            OFF2 = OFF - OFFi; //Calculate pressure and temp second order
            SENS2 = SENS - SENSi;

            self.TEMP = Some((temperature - Ti) as i32);

            self.P = Some((((d1_pres as i64 * SENS2) / (2097152 as i64) - OFF2) / 8192) as i32);
            Some(())
        }

        pub fn pressure(&self) -> Option<Pa> {
            return self.P.map(|p| Pa::new(p as f32 * 10.0));
        }

        pub fn temperature(&self) -> Option<f32> {
            return self.TEMP.map(|t| t as f32 / 100.0);
        }

        // The pressure sensor measures absolute pressure, so it will measure the atmospheric pressure + water pressure
        // We subtract the atmospheric pressure to calculate the depth with only the water pressure
        // The average atmospheric pressure of 101300 pascal is used for the calcuation, but atmospheric pressure varies
        // If the atmospheric pressure is not 101300 at the time of reading, the depth reported will be offset
        // In order to calculate the correct depth, the actual atmospheric pressure should be measured once in air, and
        // that value should subtracted for subsequent depth calculations.
        pub fn depth(&self) -> Result<Option<f32>, &str> {
            let pressure = match self.pressure() {
                Some(p) => p,
                None => return Ok(None),
            };
            let pressure_below: Pa = pressure - SURFACE_PA;
            if pressure_below.to_f32() < 0.0 {
                return Err("Use altitude instead.");
            }
            let fluid_density_10_m: f32 = (self.fluid_density as f32) * 9.80665;
            return Ok(Some(
                (pressure_below / fluid_density_10_m).to_bar().to_f32() * 10.0,
            ));
        }

        pub fn altitude(&self) -> Result<Option<f32>, &str> {
            let pressure = match self.pressure() {
                Some(p) => p,
                None => return Ok(None),
            };
            let p: f32 = pressure.to_pa() / SURFACE_PA.to_pa();
            let pow: f32 = 0.190284;
            let ratio = 1.0 - powf(p, pow);
            return Ok(Some(ratio * ALT_PER_FOOT * FEET_TO_METERS));
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
}
