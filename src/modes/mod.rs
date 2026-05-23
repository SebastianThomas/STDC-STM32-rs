use core::sync::atomic::{AtomicU8, Ordering};
use core::{cell::RefCell, time::Duration};

use cortex_m::interrupt::{Mutex, free};
use stdc_diving_algorithms::{
    dive::StopSchedule,
    pressure_unit::{Pa, Pressure},
};
use stm32l4xx_hal::{pac::TIM5, timer::Timer};

use stdc_stm32_rs::components::display::{DisplayState, LedDisplay, MAX_STOP_NUMS};
#[cfg(feature = "bluetooth")]
pub mod bluetooth;
pub mod dive;
pub mod surface;

pub const POWER_CUT_UNSAFE_TASK_RUNNING: u8 = 1 << 0;
#[cfg(feature = "bluetooth")]
pub const POWER_CUT_UNSAFE_FW_TRANSFER: u8 = 1 << 1;
pub const POWER_CUT_UNSAFE_FLASH_WRITE: u8 = 1 << 2;

static POWER_CUT_UNSAFE_MASK: AtomicU8 = AtomicU8::new(0);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AppMode {
    Surface,
    Dive,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SurfaceModeExit {
    Dive(Pa),
}

pub static DISPLAY_STATE: Mutex<RefCell<DisplayState>> =
    Mutex::new(RefCell::new(DisplayState::default()));

pub fn display_set_depth<P: Pressure>(depth: P) {
    free(|cs| {
        let mut display_state = DISPLAY_STATE.borrow(cs).borrow_mut();
        display_state.depth = depth.to_msw();
    });
}

pub fn display_set_dive_time(time: Duration) {
    free(|cs| {
        let mut display_state = DISPLAY_STATE.borrow(cs).borrow_mut();
        display_state.dive_time = time;
    });
}

pub fn display_set_stop_schedule(stops: StopSchedule<MAX_STOP_NUMS>) {
    free(|cs| {
        let mut display_state = DISPLAY_STATE.borrow(cs).borrow_mut();
        display_state.stop_schedule = Ok(stops);
    });
}

pub fn display_refresh<D: LedDisplay>(display: &mut D) -> Result<(), D::Error> {
    free(|cs| {
        let display_state = DISPLAY_STATE.borrow(cs).borrow();
        display.refresh_with_state(&display_state)
    })
}

pub fn millis_tim5() -> u32 {
    Timer::<TIM5>::count()
}

pub fn millis_tim5_since(start: u32) -> u32 {
    Timer::<TIM5>::count().wrapping_sub(start)
}

pub fn power_cut_mark_unsafe(flag: u8) {
    POWER_CUT_UNSAFE_MASK.fetch_or(flag, Ordering::Relaxed);
    crate::sync_power_cut_indicator();
}

pub fn power_cut_mark_safe(flag: u8) {
    POWER_CUT_UNSAFE_MASK.fetch_and(!flag, Ordering::Relaxed);
    crate::sync_power_cut_indicator();
}

pub fn power_cut_unsafe_mask() -> u8 {
    POWER_CUT_UNSAFE_MASK.load(Ordering::Relaxed)
}

pub fn is_power_cut_safe() -> bool {
    power_cut_unsafe_mask() == 0
}

/// Helper to perform a flash write.
/// - Mark the device unsafe for power-cut,
/// - Measuring the operation with the benchmarking harness, and logg the sample.
///
/// * `op`` closure should perform the actual flash operation
pub fn flash_write_and_measure<T, FOp>(label: &'static str, op: FOp) -> T
where
    FOp: FnOnce() -> T,
{
    with_power_cut_unsafe_measure_and_log(POWER_CUT_UNSAFE_FLASH_WRITE, label, op)
}

/// Generic helper to mark a given `flag` unsafe for power-cut, measure and log the operation,
/// then mark the flag safe again.
pub fn with_power_cut_unsafe_measure_and_log<T, FOp>(flag: u8, label: &'static str, op: FOp) -> T
where
    FOp: FnOnce() -> T,
{
    power_cut_mark_unsafe(flag);
    let res = crate::benchmarking::measure_and_log(label, op);
    power_cut_mark_safe(flag);
    res
}
