use core::cell::RefCell;

use cortex_m::interrupt::Mutex;
use stm32l4xx_hal::{
    delay::Delay,
    hal::blocking::i2c::{Write, WriteRead},
};

pub const MAX17262_I2C_ADDRESS: u8 = 0x36;

/** Reported Capacity Register */
const REG_REP_CAP: u8 = 0x05;
/** Reported State Of Charge Register */
const REG_REP_SOC: u8 = 0x06;
/** Temperature Register */
const REG_TEMP: u8 = 0x08;
/** Cell Voltage Register */
const REG_VCELL: u8 = 0x09;
/** Current Register */
const REG_CURRENT: u8 = 0x0A;
/** Reported Full Capacity Register */
const REG_REP_FULL_CAP: u8 = 0x10;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Max17262Variant {
    R,
    H,
}

impl Max17262Variant {
    fn capacity_lsb_mah(self) -> f32 {
        match self {
            Self::R => 0.5,
            Self::H => 0.1667,
        }
    }

    fn current_lsb_ma(self) -> f32 {
        match self {
            Self::R => 0.15625,
            Self::H => 0.052083,
        }
    }
}

pub trait BatteryStatus {
    fn read_snapshot(&mut self) -> Result<BatterySnapshot, BatteryStatusError>;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BatterySnapshot {
    pub state_of_charge_percent: f32,
    pub voltage_mv: u32,
    pub current_ma: f32,
    pub temperature_c: f32,
    pub remaining_capacity_mah: f32,
    pub full_capacity_mah: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BatteryStatusError {
    I2cWrite,
    I2cWriteRead,
}

/**
* Given Layout: MAX17262_REWL+T
*/
pub struct BatteryStatusI2C<'a, I> {
    interface: I,
    _delay: &'a Mutex<RefCell<Delay>>,
    address: u8,
    variant: Max17262Variant,
}

impl<I: Write + WriteRead> BatteryStatus for BatteryStatusI2C<'_, I> {
    fn read_snapshot(&mut self) -> Result<BatterySnapshot, BatteryStatusError> {
        self.read_snapshot()
    }
}

impl<'a, I: Write + WriteRead> BatteryStatusI2C<'a, I> {
    pub fn new(
        i2c: I,
        delay: &'a Mutex<RefCell<Delay>>,
        variant: Max17262Variant,
    ) -> Self {
        Self::with_address(i2c, delay, MAX17262_I2C_ADDRESS, variant)
    }

    pub fn with_address(
        i2c: I,
        delay: &'a Mutex<RefCell<Delay>>,
        address: u8,
        variant: Max17262Variant,
    ) -> Self {
        Self {
            interface: i2c,
            _delay: delay,
            address,
            variant,
        }
    }

    pub fn free(self) -> I {
        self.interface
    }

    pub fn read_snapshot(&mut self) -> Result<BatterySnapshot, BatteryStatusError> {
        let rep_soc = self.read_register_u16(REG_REP_SOC)?;
        let vcell = self.read_register_u16(REG_VCELL)?;
        let current = self.read_register_i16(REG_CURRENT)?;
        let temperature = self.read_register_i16(REG_TEMP)?;
        let rep_cap = self.read_register_u16(REG_REP_CAP)?;
        let full_cap_rep = self.read_register_u16(REG_REP_FULL_CAP)?;

        Ok(BatterySnapshot {
            state_of_charge_percent: Self::decode_soc_percent(rep_soc),
            voltage_mv: Self::decode_voltage_mv(vcell),
            current_ma: Self::decode_current_ma(current, self.variant),
            temperature_c: Self::decode_temperature_c(temperature),
            remaining_capacity_mah: Self::decode_capacity_mah(rep_cap, self.variant),
            full_capacity_mah: Self::decode_capacity_mah(full_cap_rep, self.variant),
        })
    }

    fn read_register_u16(&mut self, register: u8) -> Result<u16, BatteryStatusError> {
        let mut bytes = [0u8; 2];
        self.interface
            .write_read(self.address, &[register], &mut bytes)
            .map_err(|_| BatteryStatusError::I2cWriteRead)?;
        Ok(u16::from_le_bytes(bytes))
    }

    fn read_register_i16(&mut self, register: u8) -> Result<i16, BatteryStatusError> {
        self.read_register_u16(register).map(|raw| raw as i16)
    }

    pub fn write_register_u16(
        &mut self,
        register: u8,
        value: u16,
    ) -> Result<(), BatteryStatusError> {
        let raw = value.to_le_bytes();
        self.interface
            .write(self.address, &[register, raw[0], raw[1]])
            .map_err(|_| BatteryStatusError::I2cWrite)
    }

    fn decode_soc_percent(raw: u16) -> f32 {
        raw as f32 / 256.0
    }

    fn decode_voltage_mv(raw: u16) -> u32 {
        let microvolts = (raw as u32 * 625) / 8;
        microvolts / 1_000
    }

    fn decode_current_ma(raw: i16, variant: Max17262Variant) -> f32 {
        raw as f32 * variant.current_lsb_ma()
    }

    fn decode_temperature_c(raw: i16) -> f32 {
        raw as f32 / 256.0
    }

    fn decode_capacity_mah(raw: u16, variant: Max17262Variant) -> f32 {
        raw as f32 * variant.capacity_lsb_mah()
    }
}
