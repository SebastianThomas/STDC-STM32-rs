use core::{cell::RefCell, time::Duration};

use cortex_m::interrupt::{Mutex, free};
use stm32l4xx_hal::{pac::TIM2, timer::Timer};
use thalmann::{
    dive::StopSchedule,
    pressure_unit::{Pa, Pressure},
};

use stdc_stm32_rs::components::display::{DisplayState, LedDisplay, MAX_STOP_NUMS};

pub mod bluetooth;
pub mod dive;
pub mod surface;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AppMode {
    Surface,
    Dive,
    Bluetooth,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SurfaceModeExit {
    Dive(Pa),
    Bluetooth(Pa),
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

pub fn millis_tim2() -> u32 {
    Timer::<TIM2>::count()
}

pub fn millis_tim2_since(start: u32) -> u32 {
    Timer::<TIM2>::count().wrapping_sub(start)
}
