use core::time::Duration;

use thalmann::{
    DINC, calc_deco_schedule,
    dive::StopSchedule,
    gas::{AIR, GasMix, TissuesLoading},
    mptt,
    pressure_unit::{Pa, Pressure, msw},
    thalmann::DecoSettings,
};

use super::spi_utils::DetailsError;

pub mod spi;
pub mod ssd1353;

pub use spi::SpiDisplay;

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

pub trait LedDisplay {
    type Error: DetailsError;
    fn show_splashscreen(&mut self, text: &[u8]) -> Result<(), Self::Error>;
    fn refresh_with_state(&mut self, state: &DisplayState) -> Result<(), Self::Error>;
}
