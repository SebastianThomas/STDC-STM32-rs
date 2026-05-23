use core::time::Duration;

use rtt_target::rprintln;
use stdc_diving_algorithms::{
    deco_algorithm::{DecoSettings, calc_deco_schedule},
    gas::{GasMix, TissuesLoading},
    pressure_unit::{Pa, Pressure, msw},
    setup::NUM_TISSUES,
};

use stdc_stm32_rs::algorithms::profile_emulation::EmulationDecoOverlay;
use stdc_stm32_rs::benchmarking;
use stdc_stm32_rs::components::display::LedDisplay;
use stdc_stm32_rs::components::spi_utils::DetailsError;

use crate::modes::{
    display_refresh, display_set_dive_time, display_set_stop_schedule, millis_tim5,
    millis_tim5_since,
};

const DIVE_TIME_UPDATE_INTERVAL_MILLIS: u32 = 1_000;
const DIVE_DECO_UPDATE_MIN_INTERVAL_MILLIS: u32 = 5_000;
const DIVE_DECO_UPDATE_MAX_INTERVAL_MILLIS: u32 = 30_000;
const DIVE_DECO_SPEED_REFERENCE_M_PER_S: f32 = 0.30;
const DIVE_DECO_DEPTH_REFERENCE_M: f32 = 40.0;
const DIVE_O2TOX_UPDATE_MIN_INTERVAL_MILLIS: u32 = 10_000;
const DIVE_O2TOX_UPDATE_MAX_INTERVAL_MILLIS: u32 = 120_000;
const DIVE_O2TOX_DEPTH_REFERENCE_M: f32 = 60.0;
const DIVE_DISPLAY_REFRESH_INTERVAL_MILLIS: u32 = 250;

pub struct DiveTaskState {
    last_dive_time_update_millis: u32,
    last_deco_update_millis: u32,
    last_deco_update_depth: msw,
    last_display_refresh_millis: u32,
}

impl DiveTaskState {
    pub fn new(start_millis: u32, start_depth: msw) -> Self {
        Self {
            last_dive_time_update_millis: start_millis,
            last_deco_update_millis: start_millis,
            last_deco_update_depth: start_depth,
            last_display_refresh_millis: start_millis,
        }
    }
}

fn lerp_u32(min_value: u32, max_value: u32, factor: f32) -> u32 {
    let factor = factor.clamp(0.0, 1.0);
    let span = max_value.abs_diff(min_value) as f32;
    (min_value as f32 + span * factor + 0.5) as u32
}

pub(crate) fn deco_update_interval_millis(
    current_depth: msw,
    previous_depth: msw,
    elapsed_millis: u32,
) -> u32 {
    let depth_m = current_depth.to_f32().max(0.0);
    let depth_factor = (depth_m / DIVE_DECO_DEPTH_REFERENCE_M).clamp(0.0, 1.0);
    let depth_interval = lerp_u32(
        DIVE_DECO_UPDATE_MAX_INTERVAL_MILLIS,
        DIVE_DECO_UPDATE_MIN_INTERVAL_MILLIS,
        depth_factor,
    );

    let depth_delta_m = (current_depth.to_f32() - previous_depth.to_f32()).abs();
    let elapsed_seconds = (elapsed_millis.max(1) as f32) / 1000.0;
    let speed_m_per_s = depth_delta_m / elapsed_seconds;
    let speed_factor = (speed_m_per_s / DIVE_DECO_SPEED_REFERENCE_M_PER_S).clamp(0.0, 1.0);
    let speed_interval = lerp_u32(
        DIVE_DECO_UPDATE_MAX_INTERVAL_MILLIS,
        DIVE_DECO_UPDATE_MIN_INTERVAL_MILLIS,
        speed_factor,
    );

    depth_interval.min(speed_interval)
}

pub(crate) fn o2tox_update_interval_millis(current_depth: msw) -> u32 {
    let depth_m = current_depth.to_f32().max(0.0);
    let depth_factor = (depth_m / DIVE_O2TOX_DEPTH_REFERENCE_M).clamp(0.0, 1.0);
    lerp_u32(
        DIVE_O2TOX_UPDATE_MAX_INTERVAL_MILLIS,
        DIVE_O2TOX_UPDATE_MIN_INTERVAL_MILLIS,
        depth_factor,
    )
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
    current_depth: msw,
    current_millis: u32,
    loading: &TissuesLoading<{ NUM_TISSUES }, Pa>,
    gases: &[GasMix<f32>; NUM_GASES],
    gases_enabled: &[bool; NUM_GASES],
    deco_settings: &DecoSettings<Pa>,
) -> Option<EmulationDecoOverlay>
where
    [(); NUM_GASES * 3]: Sized,
    [(); 24 + NUM_GASES * 3]: Sized,
    [(); 4 + 24 + NUM_GASES * 3]: Sized,
    [(); 4 + (24 + NUM_GASES * 3)]: Sized,
    [(); 4 + (24 + NUM_GASES * 3) + 0]: Sized,
    [(); 24 + NUM_GASES * 3 + 0]: Sized,
{
    let elapsed_millis = current_millis.wrapping_sub(state.last_deco_update_millis);
    let next_interval = benchmarking::measure_and_log("dive.deco_schedule.rate", || {
        deco_update_interval_millis(current_depth, state.last_deco_update_depth, elapsed_millis)
    });
    if elapsed_millis < next_interval {
        #[cfg(feature = "online_benchmarking")]
        benchmarking::log_decision(
            "dive.deco_schedule.rate",
            false,
            Some(next_interval - elapsed_millis),
        );
        return None;
    }

    state.last_deco_update_millis = current_millis;
    state.last_deco_update_depth = current_depth;

    let result = benchmarking::measure_and_log("dive.deco_schedule", || {
        calc_deco_schedule(loading, gases, gases_enabled, deco_settings)
    });

    match result {
        Ok(stops) => {
            let overlay = stops
                .first_stop()
                .map(|stop| {
                    let hold_time = stops
                        .get_deco_tts(&stdc_diving_algorithms::dive::get_ascent_rate_per_meter(9));
                    let hold_samples =
                        hold_time.as_millis().saturating_add(999).div_euclid(1_000) as usize;
                    EmulationDecoOverlay {
                        stop_depth_pa: stop.depth().to_pa().to_f32(),
                        hold_samples: hold_samples.max(1),
                    }
                })
                .or(Some(EmulationDecoOverlay {
                    stop_depth_pa: 0.0,
                    hold_samples: 0,
                }));
            display_set_stop_schedule(stops);
            rprintln!("Dive deco schedule updated");
            #[cfg(feature = "online_benchmarking")]
            benchmarking::log_decision("dive.deco_schedule.rate", true, None);
            overlay
        }
        Err(err) => {
            rprintln!("Got error while calculating deco schedule: {:?}.", err);
            None
        }
    }
}

pub fn refresh_display_if_due<D: LedDisplay>(state: &mut DiveTaskState, display: &mut D) -> bool {
    if millis_tim5_since(state.last_display_refresh_millis) < DIVE_DISPLAY_REFRESH_INTERVAL_MILLIS {
        return false;
    }

    state.last_display_refresh_millis = millis_tim5();

    let result = benchmarking::measure_and_log("dive.display_refresh", || display_refresh(display));

    match result {
        Ok(_) => true,
        Err(e) => {
            rprintln!("Failed refreshing display.");
            rprintln!("Display error details: {:?}", e.details());
            false
        }
    }
}
