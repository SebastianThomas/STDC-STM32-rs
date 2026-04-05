use core::{cell::RefCell, time::Duration};

use cortex_m::interrupt::{Mutex, free};
use rtt_target::rprintln;
use thalmann::{
    calc_deco_schedule,
    gas::{GasMix, TissuesLoading},
    mptt::NUM_TISSUES,
    pressure_unit::Pa,
    thalmann::DecoSettings,
};

use stdc_stm32_rs::components::spi_utils::DetailsError;
use stdc_stm32_rs::components::{display::LedDisplay, flash::Flash, uart_log::ExternalLogger};

use crate::modes::{
    display_refresh, display_set_dive_time, display_set_stop_schedule, millis_tim2,
    millis_tim2_since,
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

pub fn update_dive_time_if_due(
    state: &mut DiveTaskState,
    dive_start_millis: u32,
    logger: &Mutex<RefCell<impl ExternalLogger>>,
) -> bool {
    if millis_tim2_since(state.last_dive_time_update_millis) < DIVE_TIME_UPDATE_INTERVAL_MILLIS {
        return false;
    }

    let current_millis = millis_tim2();
    let duration_since_start =
        Duration::from_millis((current_millis.wrapping_sub(dive_start_millis)) as u64);
    display_set_dive_time(duration_since_start);
    state.last_dive_time_update_millis = current_millis;
    log_bytes(logger, b"Dive time updated");
    true
}

pub fn update_deco_schedule_if_due<const NUM_GASES: usize, F: Flash, L: ExternalLogger>(
    state: &mut DiveTaskState,
    loading: &TissuesLoading<NUM_TISSUES, Pa>,
    gases: &[GasMix<f32>; NUM_GASES],
    deco_settings: &DecoSettings<Pa>,
    flash: &mut F,
    logger: &Mutex<RefCell<L>>,
) -> bool
where
    [(); NUM_GASES * 3]: Sized,
    [(); 24 + NUM_GASES * 3]: Sized,
    [(); 4 + 24 + NUM_GASES * 3]: Sized,
    [(); 4 + (24 + NUM_GASES * 3)]: Sized,
    [(); 4 + (24 + NUM_GASES * 3) + 0]: Sized,
    [(); 24 + NUM_GASES * 3 + 0]: Sized,
{
    if millis_tim2_since(state.last_deco_update_millis) < DIVE_DECO_UPDATE_INTERVAL_MILLIS {
        return false;
    }

    state.last_deco_update_millis = millis_tim2();

    match calc_deco_schedule(loading, gases, deco_settings) {
        Ok(stops) => {
            display_set_stop_schedule(stops);
            log_bytes(logger, b"Dive deco schedule updated");
            true
        }
        Err(err) => {
            rprintln!("Got error while calculating deco schedule: {:?}.", err);
            let _ = flash;
            false
        }
    }
}

pub fn refresh_display_if_due<D: LedDisplay, L: ExternalLogger>(
    state: &mut DiveTaskState,
    display: &mut D,
    logger: &Mutex<RefCell<L>>,
) -> bool {
    if millis_tim2_since(state.last_display_refresh_millis) < DIVE_DISPLAY_REFRESH_INTERVAL_MILLIS {
        return false;
    }

    state.last_display_refresh_millis = millis_tim2();

    match display_refresh(display) {
        Ok(_) => true,
        Err(e) => {
            log_bytes(logger, b"Failed refreshing display.");
            log_bytes(logger, e.details().as_bytes());
            false
        }
    }
}

fn log_bytes<L: ExternalLogger>(logger: &Mutex<RefCell<L>>, bytes: &[u8]) {
    free(|cs| {
        if let Err(_) = logger.borrow(cs).borrow_mut().log_bytes(bytes) {
            rprintln!("Failed logging bytes to UART");
        }
    });
}
