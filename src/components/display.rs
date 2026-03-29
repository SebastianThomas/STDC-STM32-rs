use core::time::Duration;

use stm32l4xx_hal::{hal::digital::v2::OutputPin, spi::Spi};
use thalmann::{
    DINC, calc_deco_schedule,
    dive::StopSchedule,
    gas::{AIR, GasMix, TissuesLoading},
    mptt,
    pressure_unit::{Pa, Pressure, msw},
    thalmann::DecoSettings,
};

use super::spi_utils::SpiError;

pub const MAX_STOP_NUMS: usize = 16;
pub const MAX_STOP_DEPTH: f32 = MAX_STOP_NUMS as f32 * DINC.to_msw().to_f32();
pub struct DisplayState {
    pub depth: msw,
    pub dive_time: Duration,
    pub stop_schedule: Result<StopSchedule<MAX_STOP_NUMS>, &'static str>,
}

pub const ZERO_LOADING_AIR: TissuesLoading<{ mptt::NUM_TISSUES }, Pa> =
    TissuesLoading::new(msw(0.0).to_pa(), &AIR);

impl const Default for DisplayState {
    fn default() -> Self {
        DisplayState {
            depth: msw(0.0),
            dive_time: Duration::from_millis(0),
            stop_schedule: Ok(StopSchedule::default()),
        }
    }
}

impl DisplayState {
    pub fn new<const NUM_GASES: usize>(
        gases: &[GasMix<f32>; NUM_GASES],
        deco_settings: &DecoSettings<Pa>,
    ) -> Self {
        DisplayState {
            depth: msw(0.0),
            dive_time: Duration::from_millis(0),
            stop_schedule: calc_deco_schedule::<MAX_STOP_NUMS, NUM_GASES>(
                &ZERO_LOADING_AIR,
                gases,
                deco_settings,
            ),
        }
    }
}

pub struct SpiDisplay<SPI, PINS, EN: OutputPin, RST: OutputPin, NDC: OutputPin> {
    power_enable: EN,
    spi: Spi<SPI, PINS>,
    spi_reset: RST,
    not_data_command: NDC,

    enabled: bool,
}

impl<SPI, PINS, EN: OutputPin, RST: OutputPin, NDC: OutputPin> SpiDisplay<SPI, PINS, EN, RST, NDC> {
    pub fn new(
        power_enable: EN,
        spi: Spi<SPI, PINS>,
        spi_reset: RST,
        not_data_command: NDC,
    ) -> SpiDisplay<SPI, PINS, EN, RST, NDC> {
        SpiDisplay {
            power_enable,
            spi,
            spi_reset,
            not_data_command,

            enabled: false,
        }
    }

    pub fn shutoff(&mut self) -> Result<bool, SpiError> {
        if !self.enabled {
            return Ok(false);
        }
        let _ = self.power_enable.set_low();
        Ok(true)
    }

    pub fn turn_on(&mut self) -> Result<bool, SpiError> {
        if self.enabled {
            return Ok(false);
        }
        let _ = self.power_enable.set_high();
        Ok(true)
    }

    pub fn reset() -> Result<(), SpiError> {
        todo!()
    }
}
