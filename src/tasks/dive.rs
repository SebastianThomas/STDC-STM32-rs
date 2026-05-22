use core::time::Duration;

use rtt_target::rprintln;
use stdc_diving_algorithms::{
    deco_algorithm::{DecoSettings, calc_deco_schedule},
    gas::{GasMix, TissuesLoading},
    pressure_unit::Pa,
    setup::NUM_TISSUES,
};

use stdc_stm32_rs::benchmarking;
use stdc_stm32_rs::components::display::LedDisplay;
use stdc_stm32_rs::components::spi_utils::DetailsError;

use crate::modes::{
    display_refresh, display_set_dive_time, display_set_stop_schedule, millis_tim5,
    millis_tim5_since,
};

const DIVE_TIME_UPDATE_INTERVAL_MILLIS: u32 = 1_000;
const DIVE_DECO_UPDATE_INTERVAL_MILLIS: u32 = 5_000;
const DIVE_DISPLAY_REFRESH_INTERVAL_MILLIS: u32 = 250;

pub struct DiveTaskState {
    last_dive_time_update_millis: u32,
    last_deco_update_millis: u32,
    last_display_refresh_millis: u32,
}

impl DiveTaskState {
    pub fn new(start_millis: u32) -> Self {
        Self {
            last_dive_time_update_millis: start_millis,
            last_deco_update_millis: start_millis,
            last_display_refresh_millis: start_millis,
        }
    }
}

pub fn update_dive_time_if_due(state: &mut DiveTaskState, dive_start_millis: u32) -> bool {
    if millis_tim5_since(state.last_dive_time_update_millis) < DIVE_TIME_UPDATE_INTERVAL_MILLIS {
        return false;
    }

    let current_millis = millis_tim5();
    let duration_since_start =
        Duration::from_millis((current_millis.wrapping_sub(dive_start_millis)) as u64);
    display_set_dive_time(duration_since_start);
    state.last_dive_time_update_millis = current_millis;
    rprintln!("Dive time updated");
    true
}

pub fn update_deco_schedule_if_due<const NUM_GASES: usize>(
    state: &mut DiveTaskState,
    loading: &TissuesLoading<{ NUM_TISSUES }, Pa>,
    gases: &[GasMix<f32>; NUM_GASES],
    gases_enabled: &[bool; NUM_GASES],
    deco_settings: &DecoSettings<Pa>,
) -> bool
where
    [(); NUM_GASES * 3]: Sized,
    [(); 24 + NUM_GASES * 3]: Sized,
    [(); 4 + 24 + NUM_GASES * 3]: Sized,
    [(); 4 + (24 + NUM_GASES * 3)]: Sized,
    [(); 4 + (24 + NUM_GASES * 3) + 0]: Sized,
    [(); 24 + NUM_GASES * 3 + 0]: Sized,
{
    if millis_tim5_since(state.last_deco_update_millis) < DIVE_DECO_UPDATE_INTERVAL_MILLIS {
        return false;
    }

    state.last_deco_update_millis = millis_tim5();

    let (result, sample) = benchmarking::measure("dive.deco_schedule", || {
        calc_deco_schedule(loading, gases, gases_enabled, deco_settings)
    });
    benchmarking::log_sample(&sample);

    match result {
        Ok(stops) => {
            display_set_stop_schedule(stops);
            rprintln!("Dive deco schedule updated");
            true
        }
        Err(err) => {
            rprintln!("Got error while calculating deco schedule: {:?}.", err);
            false
        }
    }
}

pub fn refresh_display_if_due<D: LedDisplay>(state: &mut DiveTaskState, display: &mut D) -> bool {
    if millis_tim5_since(state.last_display_refresh_millis) < DIVE_DISPLAY_REFRESH_INTERVAL_MILLIS {
        return false;
    }

    state.last_display_refresh_millis = millis_tim5();

    let (result, sample) =
        benchmarking::measure("dive.display_refresh", || display_refresh(display));
    benchmarking::log_sample(&sample);

    match result {
        Ok(_) => true,
        Err(e) => {
            rprintln!("Failed refreshing display.");
            rprintln!("Display error details: {:?}", e.details());
            false
        }
    }
}
