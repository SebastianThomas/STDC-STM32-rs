use core::future::Future;

#[cfg(all(target_os = "none", feature = "online_benchmarking"))]
use core::cell::RefCell;

#[cfg(all(target_os = "none", feature = "online_benchmarking"))]
use cortex_m::interrupt::{Mutex, free};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BenchmarkSample {
    pub label: &'static str,
    pub cycles: u64,
    pub nanos: u64,
}

#[cfg(all(target_os = "none", feature = "online_benchmarking"))]
const CLOCK_HZ: u64 = 16_000_000;

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
        "bench_sample,label={},cycles={},nanos={}",
        sample.label,
        sample.cycles,
        sample.nanos
    );
}

#[cfg(all(target_os = "none", feature = "online_benchmarking"))]
pub fn log_session_start(session_name: &'static str) {
    rtt_target::rprintln!("bench_session,name={}", session_name);
}

#[cfg(all(target_os = "none", feature = "online_benchmarking"))]
pub fn log_schema(schema_name: &'static str) {
    rtt_target::rprintln!("bench_schema,name={}", schema_name);
}

#[cfg(all(target_os = "none", not(feature = "online_benchmarking")))]
pub fn log_sample(_sample: &BenchmarkSample) {}

#[cfg(all(not(target_os = "none"), test))]
pub fn log_sample(sample: &BenchmarkSample) {
    println!(
        "bench_sample,label={},cycles={},nanos={}",
        sample.label, sample.cycles, sample.nanos
    );
}

#[cfg(all(not(target_os = "none"), test))]
pub fn log_session_start(session_name: &'static str) {
    println!("bench_session,name={}", session_name);
}

#[cfg(all(not(target_os = "none"), test))]
pub fn log_schema(schema_name: &'static str) {
    println!("bench_schema,name={}", schema_name);
}

#[cfg(all(not(target_os = "none"), not(test)))]
pub fn log_sample(_sample: &BenchmarkSample) {}

#[cfg(all(not(target_os = "none"), not(test)))]
pub fn log_session_start(_session_name: &'static str) {}

#[cfg(all(not(target_os = "none"), not(test)))]
pub fn log_schema(_schema_name: &'static str) {}

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

/// Measure the given closure and immediately log the resulting sample.
/// Returns the closure's result.
pub fn measure_and_log<R, F>(label: &'static str, f: F) -> R
where
    F: FnOnce() -> R,
{
    let (res, sample) = measure(label, f);
    log_sample(&sample);
    res
}

/// Async variant: measure the future, log the sample, and return the result.
pub async fn measure_async_and_log<R, F>(label: &'static str, fut: F) -> R
where
    F: Future<Output = R>,
{
    let (res, sample) = measure_async(label, fut).await;
    log_sample(&sample);
    res
}

#[cfg(all(target_os = "none", feature = "online_benchmarking"))]
pub fn log_decision(label: &'static str, due: bool, skipped_ms: Option<u32>) {
    if due {
        rtt_target::rprintln!("bench_decision,label={},due=true", label);
    } else {
        rtt_target::rprintln!(
            "bench_decision,label={},due=false,skipped_ms={}",
            label,
            skipped_ms.unwrap_or(0)
        );
    }
}

#[cfg(all(not(target_os = "none"), test))]
pub fn log_decision(label: &'static str, due: bool, skipped_ms: Option<u32>) {
    if due {
        println!("bench_decision,label={},due=true", label);
    } else {
        println!(
            "bench_decision,label={},due=false,skipped_ms={}",
            label,
            skipped_ms.unwrap_or(0)
        );
    }
}

#[cfg(all(not(target_os = "none"), not(test)))]
pub fn log_decision(_label: &'static str, _due: bool, _skipped_ms: Option<u32>) {}

#[cfg(all(target_os = "none", not(feature = "online_benchmarking")))]
pub fn log_decision(_label: &'static str, _due: bool, _skipped_ms: Option<u32>) {}

pub fn nanos_per_cycle(cycles: u64, clock_hz: u64) -> u64 {
    cycles.saturating_mul(1_000_000_000) / clock_hz
}
