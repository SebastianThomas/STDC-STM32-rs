use core::sync::atomic::{AtomicBool, Ordering};

pub fn request_bluetooth_mode<E>(
    active: &AtomicBool,
    spawn_bluetooth_mode: impl FnOnce() -> Result<(), E>,
) {
    if active
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_ok()
    {
        if spawn_bluetooth_mode().is_err() {
            active.store(false, Ordering::Release);
        }
    }
}
