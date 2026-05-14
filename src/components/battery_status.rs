mod battery_status_values;
pub use battery_status_values::Max17262Variant;
use battery_status_values::*;
use rtic_monotonics::{Monotonic, fugit::ExtU64};

use core::fmt::Debug;

use rtt_target::rprintln;
use stm32l4xx_hal::{
    hal::blocking::i2c::{Write, WriteRead},
};

use crate::stm32::Mono;

pub const MAX17262_I2C_ADDRESS: u8 = 0x36;

/** Status Register */
const REG_STATUS: u8 = 0x00;
/** .. .. .. .. .. .. .. DNR */
const REG_FSTAT: u8 = 0x3D;
/** Hibernation Config */
const REG_HIB_CFG: u8 = 0xBA;
/** Commands */
const REG_COMMAND: u8 = 0x60;
/** Reported Capacity Register */
const REG_REP_CAP: u8 = 0x05;
/** Reported State Of Charge Register in 1/256% */
const REG_REP_SOC: u8 = 0x06;
/** Temperature Register */
const REG_TEMP: u8 = 0x08;
/** Cell Voltage Register */
const REG_VCELL: u8 = 0x09;
/** Current Register */
const REG_CURRENT: u8 = 0x0A;
/** Reported Full Capacity Register */
const REG_REP_FULL_CAP: u8 = 0x10;
/** Time to Empty in 5.625s */
const REG_TTE: u8 = 0x11;

const REG_DESIGN_CAP: u8 = 0x18;
const REG_ICHG_TERM: u8 = 0x1E;
const REG_VEMPTY: u8 = 0x3A;
const REG_MODEL_CFG: u8 = 0xDB;

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
pub struct BatteryStatusI2C<I> {
    interface: I,
    address: u8,
    variant: Max17262Variant,
}

impl<I: Write + WriteRead> BatteryStatus for BatteryStatusI2C<I>
where
    <I as WriteRead>::Error: Debug,
{
    fn read_snapshot(&mut self) -> Result<BatterySnapshot, BatteryStatusError> {
        self.read_snapshot()
    }
}

impl<I: Write + WriteRead> BatteryStatusI2C<I>
where
    <I as WriteRead>::Error: Debug,
{
    pub async fn new(i2c: I, variant: Max17262Variant) -> Self {
        Self::with_address(i2c, MAX17262_I2C_ADDRESS, variant).await
    }

    async fn with_address(i2c: I, address: u8, variant: Max17262Variant) -> Self {
        let mut s = Self {
            interface: i2c,
            address,
            variant,
        };
        if let Err(e) = s.create_init().await {
            rprintln!("Failed initializing battery status IC: {:?}", e);
            panic!("Failed initializing battery status IC: {:?}", e);
        }
        s
    }

    fn is_ready_for_3_2(&mut self) -> bool {
        let status_reg = self.read_register_u16(REG_STATUS);
        let status_por = status_reg.map(|s| s & 0x0002);
        if let Ok(0) = status_por {
            return true;
        }
        if let Err(e) = status_reg {
            rprintln!("Got an error while reading battery status: {:?}", e);
        }
        false
    }

    // Initialization according to https://www.analog.com/media/en/technical-documentation/user-guides/modelgauge-m5-host-side-software-implementation-guide.pdf
    async fn create_init(&mut self) -> Result<(), BatteryStatusError> {
        rprintln!("Initialize Battery Status IC");
        if self.is_ready_for_3_2() {
            return self.init_step_3(true).await;
        }
        self.init_step_1().await
    }

    async fn init_step_1(&mut self) -> Result<(), BatteryStatusError> {
        rprintln!("Initialize Battery Status IC: Full, Step 1");
        while match self.read_register_u16(REG_FSTAT) {
            Ok(s) => s & 1 == 1,
            Err(e) => {
                rprintln!("Got error while looping reading ready register: {:?}", e);
                true
            }
        } {
            self.delay_ms(10).await;
        }
        self.init_step_2().await
    }

    async fn init_step_2(&mut self) -> Result<(), BatteryStatusError> {
        rprintln!("Initialize Battery Status IC: Full, Step 1");
        // Exit Hibernate Mode
        let initial_hib_cfg = self.read_register_u16(REG_HIB_CFG)?;
        self.write_register_u16(REG_COMMAND, 0x0090)?;
        self.write_register_u16(REG_HIB_CFG, 0x0000)?;
        self.write_register_u16(REG_COMMAND, 0x0000)?;
        // EZ Config
        self.write_register_u16(REG_DESIGN_CAP, DESIGN_CAP)?;
        self.write_register_u16(REG_ICHG_TERM, ICHG_TERM)?;
        self.write_register_u16(REG_VEMPTY, VEMPTY)?;
        // Model Config
        let modelcfg = if CHARGE_VOLTAGE > 4.275 {
            0x8400
        } else {
            0x8000
        };
        self.write_register_u16(REG_MODEL_CFG, modelcfg)?;
        while match self.read_register_u16(REG_MODEL_CFG) {
            Ok(r) => r & 0x8000 != 0,
            Err(e) => {
                rprintln!("Error writing model config {:?}", e);
                true
            }
        } {
            self.delay_ms(10).await;
        }
        // Recover initial Hibernate Config
        self.write_register_u16(REG_HIB_CFG, initial_hib_cfg)?;

        self.init_step_3_1(false)
    }

    async fn init_step_3(&mut self, skip_check: bool) -> Result<(), BatteryStatusError> {
        // Step 3
        let status = self.read_register_u16(REG_STATUS)?;
        self.write_register_u16(REG_STATUS, status & 0xFFFD)?;
        // Step 3.1
        if !skip_check && !self.is_ready_for_3_2() {
            rprintln!("Battery Status IC not ready, initializing steps 1-3");
            return self.init_step_1().await;
        }
        self.init_step_3_1(true)
    }

    fn init_step_3_1(&mut self, skip_status_write: bool) -> Result<() ,BatteryStatusError> {
        if !skip_status_write {
        // Step 3
        let status = self.read_register_u16(REG_STATUS)?;
        self.write_register_u16(REG_STATUS, status & 0xFFFD)?;}
        rprintln!("Battery Status IC ready, initializing step 3");
        let cap = self.read_register_u16(REG_REP_CAP)?;
        let soc = self.read_register_u16(REG_REP_SOC)?;
        let tte = self.read_register_u16(REG_TTE)?;
        rprintln!(
            "Finished Initializing, got values: Capacity {}mAh, SOC: {}%, TTE: {}min",
            cap,
            Self::soc_to_perc(soc),
            Self::tte_to_mins(tte)
        );
        // TODO: Step 3.4: Saved Learned Parameters
        Ok(())
    }

    fn soc_to_perc(soc: u16) -> u8 {
        return (soc as u32 * 256 as u32 / 100) as u8;
    }

    fn tte_to_mins(tte: u16) -> u16 {
        const TTE_FACTOR: u16 = 5_625;
        (tte as u32 * TTE_FACTOR as u32 / 1000 as u32) as u16
    }

    pub async fn delay_ms(&self, ms: u64) {
        Mono::delay(ms.millis()).await;
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
            .map_err(|e| {
                rprintln!("Got an error: {:?}", e);
                BatteryStatusError::I2cWriteRead
            })?;
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
