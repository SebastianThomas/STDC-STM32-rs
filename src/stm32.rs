use core::task::{RawWaker, RawWakerVTable, Waker};
use rtic_monotonics::stm32_tim5_monotonic;

pub const TIM5_INPUT_CLOCK: u32 = 8_000_000;
pub const TIM5_MONO_CLOCK: u32 = 1_000_000;

stm32_tim5_monotonic!(Mono, TIM5_MONO_CLOCK);

pub fn noop_waker() -> Waker {
    unsafe fn no_op(_: *const ()) {}

    unsafe fn clone(_: *const ()) -> RawWaker {
        raw_waker()
    }

    fn raw_waker() -> RawWaker {
        RawWaker::new(core::ptr::null(), &VTABLE)
    }

    static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, no_op, no_op, no_op);

    unsafe { Waker::from_raw(raw_waker()) }
}
