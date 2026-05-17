use core::panic::PanicInfo;
use core::sync::atomic::{AtomicBool, Ordering};

use cortex_m_rt::exception;
use rtt_target::rprintln;

#[exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    rprintln!("HARDFAULT: {:?}", ef);
    loop {}
}

// Copied from panic_probe
// Panic handler for `probe-run`.
//
// When this panic handler is used, panics will make `probe-run` print a backtrace and exit with a
// non-zero status code, indicating failure. This building block can be used to run on-device
// tests.
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    static PANICKED: AtomicBool = AtomicBool::new(false);
    rprintln!("PANIC");

    cortex_m::interrupt::disable();

    // Guard against infinite recursion, just in case.
    if !PANICKED.load(Ordering::Relaxed) {
        PANICKED.store(true, Ordering::Relaxed);

        rprintln!("{}", info);
    }

    hard_fault();
}

/// Trigger a `HardFault` via `udf` instruction.
#[cfg(target_os = "none")]
pub fn hard_fault() -> ! {
    // If `UsageFault` is enabled, we disable that first, since otherwise `udf` will cause that
    // exception instead of `HardFault`.
    {
        const SHCSR: *mut u32 = 0xE000ED24usize as _;
        const USGFAULTENA: usize = 18;

        unsafe {
            let mut shcsr = core::ptr::read_volatile(SHCSR);
            shcsr &= !(1 << USGFAULTENA);
            core::ptr::write_volatile(SHCSR, shcsr);
        }
    }

    cortex_m::asm::udf();
}
