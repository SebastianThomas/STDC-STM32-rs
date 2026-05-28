use core::time::Duration;

use rtt_target::rprintln;
use stdc_diving_algorithms::{
    deco_algorithm::{DecoSettings, calc_deco_schedule},
    dive::StopSchedule,
    gas::{GasMix, TissuesLoading},
    pressure_unit::{Pa, Pressure, msw},
    setup::NUM_TISSUES,
};

use stdc_stm32_rs::algorithms::{
    profile_emulation::EmulationDecoStop,
    rate_algorithm::{DecoUpdateRateAlgorithm, FixedRateAlgorithm, RateAlgorithm},
};
use stdc_stm32_rs::benchmarking;
use stdc_stm32_rs::components::display::LedDisplay;
use stdc_stm32_rs::components::spi_utils::DetailsError;
use stdc_stm32_rs::{
    algorithms::profile_emulation::EmulationDecoOverlay, components::display::MAX_STOP_NUMS,
};

use crate::modes::{
    display_refresh, display_set_dive_time, display_set_stop_schedule, millis_tim5,
    millis_tim5_since,
};

const DIVE_TIME_UPDATE_INTERVAL_MILLIS: u32 = 1_000;
const DIVE_DISPLAY_REFRESH_INTERVAL_MILLIS: u32 = 250;

pub struct DiveTaskState {
    last_dive_time_update_millis: u32,
    last_deco_update_millis: u32,
    last_deco_update_depth: msw,
    last_display_refresh_millis: u32,
    display_refresh_rate_algorithm: FixedRateAlgorithm,
}

impl DiveTaskState {
    pub fn new(start_millis: u32, start_depth: msw) -> Self {
        Self {
            last_dive_time_update_millis: start_millis,
            last_deco_update_millis: start_millis,
            last_deco_update_depth: start_depth,
            last_display_refresh_millis: start_millis,
            display_refresh_rate_algorithm: FixedRateAlgorithm::new(
                DIVE_DISPLAY_REFRESH_INTERVAL_MILLIS,
            ),
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
    deco_rate_algorithm: &mut DecoUpdateRateAlgorithm,
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
        deco_rate_algorithm
            .next_iter(
                (state.last_deco_update_depth, elapsed_millis),
                current_depth,
            )
            .unwrap()
    });
    // rprintln!("Next interval: {:?}", next_interval);
    // rprintln!("Elapsed Milli: {:?}", elapsed_millis);
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

    let result: Result<StopSchedule<{ MAX_STOP_NUMS }>, _> =
        benchmarking::measure_and_log("dive.deco_schedule", || {
            calc_deco_schedule(loading, gases, gases_enabled, deco_settings)
        });

    match result {
        Ok(stops) => {
            let first_stop = stops.first_stop();
            let last_stop = stops
                .stops()
                .iter()
                .rev()
                .find(|stop| stop.duration().as_millis() > 0);
            rprintln!("First stop (deepest): {:?}", first_stop);
            rprintln!("Last used stop (shallowest): {:?}", last_stop);

            #[cfg(feature = "live_sim")]
            {
                // Dump tissue loadings (a few tissues) and a summary to help debug
                let max_print = 8usize.min(NUM_TISSUES);
                rprintln!("Tissue loadings dump (first {} tissues):", max_print);
                for i in 0..max_print {
                    let n2 = loading.n2[i];
                    let he = loading.he[i];
                    rprintln!(
                        " tissue {}: n2={:?}, he={:?}, total_inert={:?}",
                        i,
                        n2,
                        he,
                        (n2 + he)
                    );
                }
                // Compute max inert tissue and value
                let mut max_idx: usize = 0;
                let mut max_val = loading.n2[0] + loading.he[0];
                for i in 1..NUM_TISSUES {
                    let val = loading.n2[i] + loading.he[i];
                    if val > max_val {
                        max_val = val;
                        max_idx = i;
                    }
                }
                rprintln!(
                    "Tissue max inert at idx={} value={:?} (Pa)",
                    max_idx,
                    max_val
                );
            }
            let mut overlay = EmulationDecoOverlay {
                stops: [EmulationDecoStop::none(); MAX_STOP_NUMS],
                count: 0,
            };
            for stop in stops
                .stops()
                .iter()
                .filter(|stop| stop.duration().as_millis() > 0)
                .take(MAX_STOP_NUMS)
            {
                let hold_samples = stop
                    .duration()
                    .as_millis()
                    .saturating_add(999)
                    .div_euclid(1_000) as usize;
                overlay.stops[overlay.count] = EmulationDecoStop {
                    stop_depth_pa: stop.depth().to_pa(),
                    hold_samples: hold_samples.max(1),
                    start_sample: None,
                };
                overlay.count += 1;
            }
            display_set_stop_schedule(stops);
            rprintln!("----------");
            rprintln!(
                "Dive deco schedule updated: count={}, first_stop={:?}, last_stop={:?}",
                overlay.count,
                overlay.stops.first().copied(),
                if overlay.count == 0 {
                    None
                } else {
                    overlay.stops.get(overlay.count - 1).copied()
                }
            );
            rprintln!("----------");
            #[cfg(feature = "online_benchmarking")]
            benchmarking::log_decision("dive.deco_schedule.rate", true, None);
            Some(overlay)
        }
        Err(err) => {
            rprintln!("Got error while calculating deco schedule: {:?}.", err);
            None
        }
    }
}

pub fn refresh_display_if_due<D: LedDisplay>(state: &mut DiveTaskState, display: &mut D) -> bool {
    let elapsed_millis = millis_tim5_since(state.last_display_refresh_millis);
    let next_interval = benchmarking::measure_and_log("dive.display_refresh.rate", || {
        state
            .display_refresh_rate_algorithm
            .next_iter((), elapsed_millis)
            .unwrap()
    });

    if elapsed_millis < next_interval {
        #[cfg(feature = "online_benchmarking")]
        benchmarking::log_decision(
            "dive.display_refresh.rate",
            false,
            Some(next_interval - elapsed_millis),
        );
        return false;
    }

    state.last_display_refresh_millis = millis_tim5();

    let result = benchmarking::measure_and_log("dive.display_refresh", || display_refresh(display));

    match result {
        Ok(_) => {
            #[cfg(feature = "online_benchmarking")]
            benchmarking::log_decision("dive.display_refresh.rate", true, None);
            true
        }
        Err(e) => {
            rprintln!("Failed refreshing display.");
            rprintln!("Display error details: {:?}", e.details());
            false
        }
    }
}
