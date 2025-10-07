pub mod barometric {
    use crate::pressure_unit::*;

    pub const SURFACE_PA: Pa = Pa(101325.0);
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

    use core::prelude::rust_2024::derive;

    use libm::powf;

    // use byteorder::{BigEndian, ByteOrder};
    // use i2cdev::core::I2CDevice;
    use stm32l4xx_hal::{
        delay::Delay,
        prelude::*,
        hal::blocking::i2c::{Read, Write, WriteRead},
    };

    use crate::ms5849::barometric::*;
    use crate::pressure_unit::*;

    pub const MPL115A2_I2C_ADDR: u16 = 0x60; // appears to always be this

    const REGISTER_ADDR_PADC: u8 = 0x00;
    const REGISTER_ADDR_TADC: u8 = 0x02;
    const REGISTER_ADDR_A0: u8 = 0x04; // other coefficients follow
    const REGISTER_ADDR_START_CONVERSION: u8 = 0x12;

    /// Provides access to the MPL115A2 Temperature and Pressure Sensor
    ///
    /// http://cache.freescale.com/files/sensors/doc/data_sheet/MPL115A2.pdf
    //     pub struct MS584930BABarometerThermometer<T: I2CDevice + Sized> {
    //         pub i2cdev: T,
    //         pub coeff: MS584930BACoefficients,
    //     }

    /// In order to get either the temperature or humdity it is
    /// necessary to read several different values from the chip.
    ///
    /// These are not generally useful in and of themselves but
    /// are used for calculating temperature/pressure.  The structure
    /// is exposed externally as they *could* be useful for some
    /// unknown use case.  Generally, you shouldn't ever need
    /// to use this directly.
    ///
    /// One use case for use of this struct directly would be for
    /// getting both temperature and pressure in a single call.
    #[derive(Debug)]
    pub struct MS584930BARawReading {
        padc: u16, // 10-bit pressure ADC output value
        tadc: u16, // 10-bit pressure ADC output value
    }

    /// The sensors has several coefficients that must be used in order
    /// to calculate a correct value for pressure/temperature.
    ///
    /// This structure provides access to those.  It is usually only
    /// necessary to read these coefficients once per interaction
    /// with the acclerometer.  It does not need to be read again
    /// on each sample.
    #[derive(Debug)]
    pub struct MS584930BACoefficients {
        a0: f32,  // 16 bits, 1 sign, 12 int, 3 fractional, 0 dec pt 0 pad
        b1: f32,  // 16 bits, 1 sign, 2 int, 13 fractional, 0 dec pt 0 pad
        b2: f32,  // 16 bits, 1 sign, 1 int, 14 fractional, 0 dec pt 0 pad
        c12: f32, // 16 bits, 1 sign, 0 int, 13 fractional, 9 dec pt 0 pad
    }

    //    fn calc_coefficient(
    //        msb: u8,
    //        lsb: u8,
    //        integer_bits: i32,
    //        fractional_bits: i32,
    //        dec_pt_zero_pad: i32,
    //    ) -> f32 {
    //        // If values are less than 16 bytes, need to adjust
    //        let extrabits = 16 - integer_bits - fractional_bits - 1;
    //        let rawval: i16 = BigEndian::read_i16(&[msb, lsb]);
    //        (f32::from(rawval) / 2_f32.powi(fractional_bits + extrabits)) / 10_f32.powi(dec_pt_zero_pad)
    //    }
    //
    //    impl MS584930BACoefficients {
    //        /// Convert a slice of data values of length 8 to coefficients
    //        ///
    //        /// This should be built from a read of registers 0x04-0x0B in
    //        /// order.  This gets the raw, unconverted value of each
    //        /// coefficient.
    //        pub fn new<E: Error>(
    //            i2cdev: &mut dyn I2CDevice<Error = E>,
    //        ) -> Result<MS584930BACoefficients, E> {
    //            let mut buf: [u8; 8] = [0; 8];
    //            i2cdev.write(&[REGISTER_ADDR_A0])?;
    //            i2cdev.read(&mut buf)?;
    //            Ok(MS584930BACoefficients {
    //                a0: calc_coefficient(buf[0], buf[1], 12, 3, 0),
    //                b1: calc_coefficient(buf[2], buf[3], 2, 13, 0),
    //                b2: calc_coefficient(buf[4], buf[5], 1, 14, 0),
    //                c12: calc_coefficient(buf[6], buf[7], 0, 13, 9),
    //            })
    //        }
    //    }
    //
    //    impl MS584930BARawReading {
    //        /// Create a new reading from the provided I2C Device
    //        pub fn new<E: Error>(
    //            i2cdev: &mut dyn I2CDevice<Error = E>,
    //        ) -> Result<MS584930BARawReading, E> {
    //            // tell the chip to do an ADC read so we can get updated values
    //            i2cdev.smbus_write_byte_data(REGISTER_ADDR_START_CONVERSION, 0x00)?;
    //
    //            // maximum conversion time is 3ms
    //            wait_ms(3u16);
    //
    //            // The SMBus functions read word values as little endian but that is not
    //            // what we want
    //            let mut buf = [0_u8; 4];
    //            i2cdev.write(&[REGISTER_ADDR_PADC])?;
    //            i2cdev.read(&mut buf)?;
    //            let padc: u16 = BigEndian::read_u16(&buf) >> 6;
    //            let tadc: u16 = BigEndian::read_u16(&buf[2..]) >> 6;
    //            Ok(MS584930BARawReading { padc, tadc })
    //        }
    //
    //        /// Calculate the temperature in centrigrade for this reading
    //        pub fn temperature_celsius(&self) -> f32 {
    //            (f32::from(self.tadc) - 498.0) / -5.35 + 25.0
    //        }
    //
    //        /// Calculate the pressure in pascals for this reading
    //        pub fn pressure_kpa(&self, coeff: &MS584930BACoefficients) -> kPa {
    //            // Pcomp = a0 + (b1 + c12 * Tadc) * Padc + b2 * Tadc
    //            // Pkpa = Pcomp * ((115 - 50) / 1023) + 50
    //            let pcomp: f32 = coeff.a0
    //                + (coeff.b1 + coeff.c12 * f32::from(self.tadc)) * f32::from(self.padc)
    //                + (coeff.b2 * f32::from(self.tadc));
    //
    //            // scale has 1023 bits of range from 50 kPa to 115 kPa
    //            let pkpa: f32 = pcomp * ((115.0 - 50.0) / 1023.0) + 50.0;
    //            pkpa
    //        }
    //    }

    pub const MS5849_ADDR: u8 = 0x76;
    pub const MS5849_RESET: u8 = 0x1E;
    pub const MS5849_ADC_READ: u8 = 0x00;
    pub const MS5849_PROM_READ: u8 = 0xA0;
    pub const MS5849_CONVERT_D1_8192: u8 = 0x4A;
    pub const MS5849_CONVERT_D2_8192: u8 = 0x5A;

    pub const MS5849_30BA: u8 = 0;
    pub const MS5849_02BA: u8 = 1;
    pub const MS5849_UNRECOGNISED: u8 = 255;

    // context: https://github.com/ArduPilot/ardupilot/pull/29122#issuecomment-2877269114
    pub const MS5849_02BA_MAX_SENSITIVITY: u16 = 49000;
    pub const MS5849_02BA_30BA_SEPARATION: u16 = 37000;
    pub const MS5849_30BA_MIN_SENSITIVITY: u16 = 26000;

    #[allow(non_camel_case_types)]
    #[allow(non_snake_case)]
    pub struct MS5849<'a, I: Write + Read + WriteRead> {
        /** g/l */
        fluid_density: u16,
        _i2cPort: I, // I2c<I2C2, (PB10<Alternate<OpenDrain, 4>>, PB11<Alternate<OpenDrain, 4>>)>,
        P: Option<i32>,
        TEMP: Option<i32>,
        D1_pres: Option<u32>,
        D2_temp: Option<u32>,
        C: [u16; 8],
        delay: &'a Mutex<RefCell<Delay>>,
    }

    fn scan_i2c<I: Write>(i2c: &mut I) 
            where <I as cortex_m::prelude::_embedded_hal_blocking_i2c_Write>::Error: Debug {
        rprintln!("Scanning I2C bus...");

        for addr in 0x03..0x78 {
            // valid 7-bit I2C addresses
            // Try a zero-length write (common method to ping device)
            let result = i2c.write(addr, &[]);
            match result {
                Ok(_) => rprintln!("I2C device found at 0x{:02X}", addr),
                // Err(res) => rprintln!("No I2C device found at 0x{:02X}: {:?}", addr, res),
                Err(_) => ()
            }
        }
        rprintln!("Scan complete.");
    }

    fn write_i2c<I: Write>(i2c: &mut I, addr: u8, data: &[u8]) 
            where <I as cortex_m::prelude::_embedded_hal_blocking_i2c_Write>::Error: Debug {
        rprintln!("Writing to {:#x}: {:?}", addr, data);
        if let Err(e) = i2c.write(addr, data) {
            rprintln!("Write failed: {:?}", e);
            panic!("Write failed");
        }
        rprintln!("Successfully written to {:#x}: {:?}", addr, data);
    }

    fn read_i2c<I: Read, const NR: usize>(i2c: &mut I, addr: u8) -> [u8; NR] 
            where <I as cortex_m::prelude::_embedded_hal_blocking_i2c_Read>::Error: Debug {
        let mut result: [u8; NR] = [0; NR];
        if let Err(e) = i2c.read(addr, &mut result) {
            rprintln!("Reading failed: {:?}", e);
            panic!("Reading failed");
        }
        result
    }

    fn wait_ms(delay: &Mutex<RefCell<Delay>>, ms: u8) {
        free(|cs| delay.borrow(cs).borrow_mut().delay_ms(ms)); // Max conversion time per datasheet
    }

    impl<'a, I: Write + Read + WriteRead> MS5849<'a, I> 
            where <I as Read>::Error : Debug, <I as Write>::Error : Debug, <I as WriteRead>::Error : Debug {
        pub fn new(
            mut i2c: I,
            delay: &'a Mutex<RefCell<Delay>>,
        ) -> MS5849<'a, I> {
            let mut cal: [u16; 8] = [0; 8];

            rprintln!("Created I2C Interface");

            scan_i2c(&mut i2c);

            /* Reset and Calibrate */
            // Reset the MS5837, per datasheet
            write_i2c(&mut i2c, MS5849_ADDR, &[MS5849_RESET]);
            rprintln!("Written for the first time");

            // Read calibration values and CRC
            for i in 0..7 {
                write_i2c(&mut i2c, MS5849_ADDR, &[MS5849_PROM_READ + i * 2]);

                // let mut buf: [u8; 2] = [0; 2];
                // if let Err(e) = i2c.read(MS5849_ADDR, &mut buf) {
                //     rprintln!("Converting failed: {:?}", e);
                //     panic!("Converting failed");
                // }
                let buf: [u8; 2] = read_i2c(&mut i2c, MS5849_ADDR);
                cal[i as usize] = (buf[0] as u16) << 8 | buf[1] as u16;
            }

            // Verify that data is correct with CRC
            let crc_read: u8 = (cal[0] >> 12) as u8;
            let crc_calculated: u8 = crc4(&mut cal);

            if crc_calculated != crc_read {
                // CRC fail
                rprintln!("CRC failed");
                panic!("CRC failed");
            }

            MS5849 {
                fluid_density: 1029,
                _i2cPort: i2c,
                P: None,
                TEMP: None,
                D1_pres: None,
                D2_temp: None,
                C: cal,
                delay,
            }
        }

        pub fn measure_pressure_to_bar(&self) -> Option<Bar> {
            self.pressure().map(|pa: Pa| pa.to_bar())
        }

        fn wait_ms(&self, ms: u8) {
            wait_ms(&self.delay, ms);
        }

        pub fn read(&mut self) {
            // Request D1 conversion
            write_i2c(&mut self._i2cPort, MS5849_ADDR, &[MS5849_CONVERT_D1_8192]);

            self.wait_ms(20u8); // Max conversion time per datasheet

            write_i2c(&mut self._i2cPort, MS5849_ADDR, &[MS5849_ADC_READ]);

            let mut buf: [u8; 3] = [0; 3];
            if let Err(e) = self._i2cPort.read(MS5849_ADDR, &mut buf) {
                rprintln!("Failed converting: {:?}", e);
                panic!("Failed converting.");
            }
            let d1_pres = (buf[0] as u32) << 16 | (buf[0] as u32) << 8 | buf[0] as u32;
            self.D1_pres = Some(d1_pres);

            // Request D2 conversion
            write_i2c(&mut self._i2cPort, MS5849_ADDR, &[MS5849_CONVERT_D2_8192]);

            self.wait_ms(20u8); // Max conversion time per datasheet

            write_i2c(&mut self._i2cPort, MS5849_ADDR, &[MS5849_ADC_READ]);

            let mut buf: [u8; 3] = [0; 3];
            if let Err(e) = self._i2cPort.read(MS5849_ADDR, &mut buf) {
                rprintln!("Failed converting: {:?}", e);
                panic!("Failed converting.");
            }
            let d2_temp = (buf[0] as u32) << 16 | (buf[0] as u32) << 8 | buf[0] as u32;
            self.D2_temp = Some(d2_temp);

            self.calculate();
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
            return self.P.map(|p| Pa(p as f32 * 10.0));
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
                None => return Ok(None)
            };
            let pressure_below: Pa = pressure - SURFACE_PA;
            if pressure_below.to_f32() < 0.0 {
                return Err("Use altitude instead.")
            }
            let fluid_density_10_m: f32 = (self.fluid_density as f32) * 9.80665;
            return Ok (Some ((pressure_below / fluid_density_10_m).to_bar().0 * 10.0));
        }

        pub fn altitude(&self) -> Option<f32> {
            let pressure = self.pressure()?;
            let p: f32 = pressure.to_pa().0 / SURFACE_PA.to_pa().0;
            let pow: f32 = 0.190284;
            let ratio = 1.0 - powf(p, pow);
            return Some(ratio * ALT_PER_FOOT * FEET_TO_METERS);
        }
    }

    fn crc4(n_prom: &mut [u16; 8]) -> u8 {
        let mut n_rem = 0 as u16;

        n_prom[0] = (n_prom[0]) & 0x0FFF;
        n_prom[7] = 0;

        for i in 0..16 {
            if i % 2 == 1 {
                n_rem ^= ((n_prom[i >> 1]) & 0x00FF) as u16;
            } else {
                n_rem ^= (n_prom[i >> 1] >> 8) as u16;
            }
            for _n_bit in (1..=8).rev() {
                if n_rem & 0x8000 != 0 {
                    n_rem = (n_rem << 1) ^ 0x3000;
                } else {
                    n_rem = n_rem << 1;
                }
            }
        }
        n_rem = (n_rem >> 12) & 0x000F;
        return (n_rem ^ 0x00).try_into().unwrap();
    }
}
