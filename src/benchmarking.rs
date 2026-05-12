use core::future::Future;

#[cfg(all(target_os = "none", feature = "online_benchmarking"))]
use core::cell::RefCell;

#[cfg(all(target_os = "none", feature = "online_benchmarking"))]
use cortex_m::interrupt::{free, Mutex};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BenchmarkSample {
    pub label: &'static str,
    pub cycles: u64,
    pub nanos: u64,
}

#[cfg(all(target_os = "none", feature = "online_benchmarking"))]
const CLOCK_HZ: u64 = 32_000_000; // TODO: Check this against HSI, PLL

#[cfg(all(target_os = "none", feature = "online_benchmarking"))]
static WFI_COUNT: Mutex<RefCell<u64>> = Mutex::new(RefCell::new(0));

#[cfg(all(target_os = "none", feature = "online_benchmarking"))]
pub fn enable_cycle_counter(core: &mut cortex_m::Peripherals) {
    cortex_m::peripheral::DWT::unlock();
    core.DCB.enable_trace();
    core.DWT.set_cycle_count(0);
    core.DWT.enable_cycle_counter();
}

#[cfg(not(target_os = "none"))]
pub fn enable_cycle_counter(_core: &mut ()) {}

#[cfg(all(target_os = "none", not(feature = "online_benchmarking")))]
pub fn enable_cycle_counter(_core: &mut cortex_m::Peripherals) {}

#[cfg(all(target_os = "none", feature = "online_benchmarking"))]
pub fn measure<R>(label: &'static str, f: impl FnOnce() -> R) -> (R, BenchmarkSample) {
    let start = cortex_m::peripheral::DWT::cycle_count() as u64;
    let result = f();
    let end = cortex_m::peripheral::DWT::cycle_count() as u64;
    let cycles = end.wrapping_sub(start);

    (
        result,
        BenchmarkSample {
            label,
            cycles,
            nanos: cycles.saturating_mul(1_000_000_000) / CLOCK_HZ,
        },
    )
}

#[cfg(all(target_os = "none", not(feature = "online_benchmarking")))]
pub fn measure<R>(label: &'static str, f: impl FnOnce() -> R) -> (R, BenchmarkSample) {
    let result = f();
    (
        result,
        BenchmarkSample {
            label,
            cycles: 0,
            nanos: 0,
        },
    )
}

#[cfg(all(not(target_os = "none"), test))]
pub fn measure<R>(label: &'static str, f: impl FnOnce() -> R) -> (R, BenchmarkSample) {
    let start = std::time::Instant::now();
    let result = f();
    let nanos = start.elapsed().as_nanos() as u64;

    (
        result,
        BenchmarkSample {
            label,
            cycles: nanos,
            nanos,
        },
    )
}

#[cfg(all(not(target_os = "none"), not(test)))]
pub fn measure<R>(label: &'static str, f: impl FnOnce() -> R) -> (R, BenchmarkSample) {
    let result = f();
    (
        result,
        BenchmarkSample {
            label,
            cycles: 0,
            nanos: 0,
        },
    )
}

pub async fn measure_async<R, F>(label: &'static str, fut: F) -> (R, BenchmarkSample)
where
    F: Future<Output = R>,
{
    #[cfg(all(target_os = "none", feature = "online_benchmarking"))]
    let start = cortex_m::peripheral::DWT::cycle_count() as u64;
    #[cfg(all(not(target_os = "none"), test))]
    let start = std::time::Instant::now();
    #[cfg(all(not(target_os = "none"), not(test)))]
    let _start = ();

    let result = fut.await;

    #[cfg(all(target_os = "none", feature = "online_benchmarking"))]
    let cycles = cortex_m::peripheral::DWT::cycle_count() as u64 - start;
    #[cfg(all(target_os = "none", feature = "online_benchmarking"))]
    let nanos = cycles.saturating_mul(1_000_000_000) / CLOCK_HZ;

    #[cfg(all(target_os = "none", not(feature = "online_benchmarking")))]
    let nanos = 0;
    #[cfg(all(target_os = "none", not(feature = "online_benchmarking")))]
    let cycles = 0;

    #[cfg(all(not(target_os = "none"), test))]
    let nanos = start.elapsed().as_nanos() as u64;
    #[cfg(all(not(target_os = "none"), test))]
    let cycles = nanos;

    #[cfg(all(not(target_os = "none"), not(test)))]
    let nanos = 0;
    #[cfg(all(not(target_os = "none"), not(test)))]
    let cycles = 0;

    (
        result,
        BenchmarkSample {
            label,
            cycles,
            nanos,
        },
    )
}

#[cfg(all(target_os = "none", feature = "online_benchmarking"))]
pub fn log_sample(sample: &BenchmarkSample) {
    rtt_target::rprintln!(
        "bench {} cycles={} nanos={}",
        sample.label,
        sample.cycles,
        sample.nanos
    );
}

#[cfg(all(target_os = "none", not(feature = "online_benchmarking")))]
pub fn log_sample(_sample: &BenchmarkSample) {}

#[cfg(all(not(target_os = "none"), test))]
pub fn log_sample(sample: &BenchmarkSample) {
    println!(
        "bench {} cycles={} nanos={}",
        sample.label,
        sample.cycles,
        sample.nanos
    );
}

#[cfg(all(not(target_os = "none"), not(test)))]
pub fn log_sample(_sample: &BenchmarkSample) {}

#[cfg(all(target_os = "none", feature = "online_benchmarking"))]
pub fn record_wfi() {
    free(|cs| {
        *WFI_COUNT.borrow(cs).borrow_mut() += 1;
    });
}

#[cfg(all(target_os = "none", not(feature = "online_benchmarking")))]
pub fn record_wfi() {}

#[cfg(all(not(target_os = "none"), test))]
pub fn record_wfi() {}

#[cfg(all(not(target_os = "none"), not(test)))]
pub fn record_wfi() {}

#[cfg(all(target_os = "none", feature = "online_benchmarking"))]
pub fn wfi_count() -> u64 {
    free(|cs| *WFI_COUNT.borrow(cs).borrow())
}

#[cfg(all(target_os = "none", not(feature = "online_benchmarking")))]
pub fn wfi_count() -> u64 {
    0
}

#[cfg(all(not(target_os = "none"), test))]
pub fn wfi_count() -> u64 {
    0
}

#[cfg(all(not(target_os = "none"), not(test)))]
pub fn wfi_count() -> u64 {
    0
}

pub fn log_samples(samples: &[BenchmarkSample]) {
    for sample in samples {
        log_sample(sample);
    }
}

pub fn nanos_per_cycle(cycles: u64, clock_hz: u64) -> u64 {
    cycles.saturating_mul(1_000_000_000) / clock_hz
}