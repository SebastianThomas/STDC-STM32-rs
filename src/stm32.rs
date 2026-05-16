use core::task::{RawWaker, RawWakerVTable, Waker};
use rtic_monotonics::stm32_tim2_monotonic;

// Keep this moderate: with the current blocking workloads inside async task execution,
// a very high monotonic tick rate can cause half-period ISR starvation and trigger
// `Monotonic must have missed an interrupt!` asserts in rtic-monotonics.
pub const TIM2_MONO_CLOCK: u32 = 1_000;

stm32_tim2_monotonic!(Mono, TIM2_MONO_CLOCK);

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
