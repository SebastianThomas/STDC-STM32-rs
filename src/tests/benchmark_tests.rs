use stdc_diving_algorithms::{
    deco_algorithm::{DecoSettings, calc_deco_schedule},
    dive::DiveMeasurement,
    gas::{AIR, GasDensitySettings, MAX_GAS_DENSITY, TissuesLoading},
    o2tox::{O2ExposureType, O2ToxCalculation, O2ToxicityPercentage, calculate_toxicity_diff},
    pressure_unit::{Bar, Pa, Pressure},
    setup::NUM_TISSUES,
};

use crate::{
    algorithms::rate_algorithm::{DynamicDiffAimdRateAlgorithm, RateAlgorithm},
    benchmarking::{self, BenchmarkSample},
    dive_log_host::{
        DecoAlgorithmType, LevelState, LogDiveControlDataBlock, LogPointData, LogPointMetadata,
    },
};

use std::sync::Mutex;
use std::vec::Vec;

static REPORT_PRINT_LOCK: Mutex<()> = Mutex::new(());

fn print_rule() {
    println!("+---------------------------------------------------------------+");
}

fn print_row(content: &str) {
    println!("| {:<61} |", content);
}

fn print_title(title: &str) {
    print_rule();
    print_row(title);
    print_rule();
}

fn stats_row(
    label: &str,
    count: usize,
    min_nanos: u64,
    median_nanos: u64,
    avg_nanos: u64,
    max_nanos: u64,
) {
    print_row(&format!(
        "{:<24} | {:<6} | {:<12} | {:<12} | {:<12} | {:<12}",
        label,
        count,
        format!("{} ns", min_nanos),
        format!("{} ns", median_nanos),
        format!("{} ns", avg_nanos),
        format!("{} ns", max_nanos)
    ));
}

fn print_sample_stats(label: &str, samples: &[BenchmarkSample]) {
    if samples.is_empty() {
        stats_row(label, 0, 0, 0, 0, 0);
        return;
    }

    let mut nanos: Vec<u64> = samples.iter().map(|sample| sample.nanos).collect();
    nanos.sort_unstable();

    let mut min_nanos = u64::MAX;
    let mut max_nanos = 0u64;
    let mut total_nanos: u128 = 0;
    for sample in samples {
        min_nanos = min_nanos.min(sample.nanos);
        max_nanos = max_nanos.max(sample.nanos);
        total_nanos += sample.nanos as u128;
    }
    let avg_nanos = (total_nanos / samples.len() as u128) as u64;
    let median_nanos = if nanos.len() % 2 == 0 {
        let upper = nanos.len() / 2;
        let lower = upper - 1;
        (nanos[lower] as u128 + nanos[upper] as u128).div_euclid(2) as u64
    } else {
        nanos[nanos.len() / 2]
    };

    stats_row(
        label,
        samples.len(),
        min_nanos,
        median_nanos,
        avg_nanos,
        max_nanos,
    );
}

fn print_single_sample(sample: &BenchmarkSample) {
    let cycles_per_measurement = sample.cycles;
    print_row(&format!(
        "{:<24} | {:<14} | {:<14}",
        sample.label,
        format!("{} cyc", cycles_per_measurement),
        format!("{} ns", sample.nanos)
    ));
}

const WINDOW_SIZES: [usize; 5] = [1, 2, 4, 8, 16];
const MIN_WINDOW_SAMPLE_COUNT: usize = 30;

fn benchmark_dive_profile(name: &'static str, measurements: &[DiveMeasurement<Pa>]) {
    assert!(!measurements.is_empty());
    let n = measurements.len();

    let surface = Pa::new(101_325.0);
    let gases = [AIR];
    let deco_settings = DecoSettings {
        gas_density_settings: GasDensitySettings::Limit {
            limit: MAX_GAS_DENSITY.to_pa(),
        },
        max_deco_po2: Bar::new(1.6).to_pa(),
    };

    let mut window_reports: Vec<(
        usize,
        Vec<BenchmarkSample>,
        Vec<BenchmarkSample>,
        Vec<BenchmarkSample>,
    )> = Vec::new();
    for window_size in WINDOW_SIZES {
        let mut loading = TissuesLoading::<NUM_TISSUES, Pa>::new(surface, &AIR);
        let mut window_rate = DynamicDiffAimdRateAlgorithm::<Pa>::new::<3, 4>(
            5_000,
            20_000,
            Bar::new(0.2).to_pa(),
            Bar::new(1.0).to_pa(),
            Bar::new(0.2).to_pa(),
        );
        let mut window_start = 0usize;
        let mut loading_samples: Vec<BenchmarkSample> = Vec::new();
        let mut deco_samples: Vec<BenchmarkSample> = Vec::new();
        let mut rate_samples: Vec<BenchmarkSample> = Vec::new();
        let mut previous = measurements[0];
        let mut current_for_rate = surface;

        while window_start < n {
            let window_end = core::cmp::min(window_start + window_size, n);
            let ((), loading_sample) = benchmarking::measure(name, || {
                for measurement in &measurements[window_start..window_end] {
                    let delta_ms = measurement.time_ms.saturating_sub(previous.time_ms);
                    let delta_ms = if delta_ms > u16::MAX as usize {
                        u16::MAX
                    } else {
                        delta_ms as u16
                    };
                    loading.tick(delta_ms, measurement.depth, &AIR);
                    previous = *measurement;
                }
            });
            assert!(loading_sample.nanos > 0 || loading_sample.cycles > 0);
            loading_samples.push(loading_sample);

            let (_, deco_sample) = benchmarking::measure("deco.schedule.window", || {
                calc_deco_schedule::<16, 1>(&loading, &gases, &deco_settings)
            });
            assert!(deco_sample.nanos > 0 || deco_sample.cycles > 0);
            deco_samples.push(deco_sample);

            let ((), rate_window_sample) = benchmarking::measure("rate.algorithm.window", || {
                for measurement in &measurements[window_start..window_end] {
                    let _ = window_rate.next_iter(current_for_rate, measurement.depth);
                    current_for_rate = measurement.depth;
                }
            });
            assert!(rate_window_sample.nanos > 0 || rate_window_sample.cycles > 0);
            rate_samples.push(rate_window_sample);

            window_start = window_end;
        }

        assert!(
            loading_samples.len() >= MIN_WINDOW_SAMPLE_COUNT,
            "window_size={} yielded only {} samples; increase measurement count",
            window_size,
            loading_samples.len()
        );
        assert!(
            deco_samples.len() >= MIN_WINDOW_SAMPLE_COUNT,
            "window_size={} yielded only {} samples; increase measurement count",
            window_size,
            deco_samples.len()
        );
        assert!(
            rate_samples.len() >= MIN_WINDOW_SAMPLE_COUNT,
            "window_size={} yielded only {} samples; increase measurement count",
            window_size,
            rate_samples.len()
        );

        window_reports.push((window_size, loading_samples, deco_samples, rate_samples));
    }

    let mut flash_rate = DynamicDiffAimdRateAlgorithm::<Pa>::new::<3, 4>(
        5_000,
        20_000,
        Bar::new(0.2).to_pa(),
        Bar::new(1.0).to_pa(),
        Bar::new(0.2).to_pa(),
    );
    let (_, rate_sample) = benchmarking::measure("rate.algorithm", || {
        let mut current = surface;
        for measurement in measurements {
            let _ = flash_rate.next_iter(current, measurement.depth);
            current = measurement.depth;
        }
    });
    assert!(rate_sample.nanos > 0 || rate_sample.cycles > 0);

    let (_, o2_sample) = benchmarking::measure("o2.tox", || {
        calculate_toxicity_diff(
            &[DiveMeasurement {
                time_ms: 0,
                depth: surface,
                gas: 0,
            }],
            &gases,
            1,
            &O2ToxicityPercentage::new(0.0, 0.0),
            &O2ExposureType::Single,
            O2ToxCalculation::RevisedDHM2025,
        )
    });
    assert!(o2_sample.nanos > 0 || o2_sample.cycles > 0);

    let control_block = LogDiveControlDataBlock::<1>::new(
        1_700_000_000,
        1_234,
        1,
        1_013,
        23,
        4,
        &gases,
        DecoAlgorithmType::LinearExponential,
    );
    let (_, control_sample) =
        benchmarking::measure("log.control.serialize", || control_block.raw_bytes());
    assert!(control_sample.nanos > 0 || control_sample.cycles > 0);

    let point = LogPointData::new(
        LogPointMetadata::new(true, false, LevelState::Descending, [false; 4]),
        &DiveMeasurement {
            depth: surface,
            time_ms: 12_345,
            gas: 0,
        },
        0,
        19,
        88,
    );
    let (_, point_sample) = benchmarking::measure("log.point.serialize", || point.basic_bytes());
    assert!(point_sample.nanos > 0 || point_sample.cycles > 0);

    let _guard = REPORT_PRINT_LOCK.lock().unwrap();
    println!();
    print_title(&format!("Benchmark Report - {}", name));
    for (window_size, loading_samples, deco_samples, rate_samples) in &window_reports {
        print_row(&format!("window_size={} measurements={}", window_size, n));
        print_rule();
        print_row(&format!(
            "{:<24} | {:<6} | {:<12} | {:<12} | {:<12} | {:<12}",
            "window metric", "count", "min", "median", "avg", "max"
        ));
        print_rule();
        print_sample_stats("loading.window", loading_samples);
        print_sample_stats("deco.schedule.window", deco_samples);
        print_sample_stats("rate.algorithm.window", rate_samples);
        print_rule();
    }
    print_row(&format!(
        "{:<24} | {:<14} | {:<14}",
        "single metric", "cycles", "time"
    ));
    print_rule();
    print_single_sample(&rate_sample);
    print_single_sample(&o2_sample);
    print_single_sample(&control_sample);
    print_single_sample(&point_sample);
    print_rule();
    print_row(&format!(
        "measurements={} window_sizes={:?} min_count={}",
        n, WINDOW_SIZES, MIN_WINDOW_SAMPLE_COUNT
    ));
    print_rule();
}

fn generate_profile_small(count: usize) -> Vec<DiveMeasurement<Pa>> {
    let surface = 101_325.0;
    let max_depth_delta = 35_000.0;
    let mut out = Vec::with_capacity(count);
    for i in 0..count {
        let phase = (i % 128) as f32 / 127.0;
        let triangle = if phase <= 0.5 {
            phase * 2.0
        } else {
            (1.0 - phase) * 2.0
        };
        out.push(DiveMeasurement {
            depth: Pa::new(surface + triangle * max_depth_delta),
            time_ms: i * 15_000,
            gas: 0,
        });
    }
    out
}

fn generate_profile_large(count: usize) -> Vec<DiveMeasurement<Pa>> {
    let surface = 101_325.0;
    let max_depth_delta = 50_000.0;
    let mut out = Vec::with_capacity(count);
    for i in 0..count {
        let phase = (i % 192) as f32 / 191.0;
        let triangle = if phase <= 0.5 {
            phase * 2.0
        } else {
            (1.0 - phase) * 2.0
        };
        out.push(DiveMeasurement {
            depth: Pa::new(surface + triangle * max_depth_delta),
            time_ms: i * 20_000,
            gas: 0,
        });
    }
    out
}

fn generate_profile_deep(count: usize) -> Vec<DiveMeasurement<Pa>> {
    const SURFACE: Pa = Pa::new(101_325.0);
    // Bar difference from surface to bottom
    const BOTTOM_DEPTH_DELTA: Pa = Bar::new(9.0).to_pa();
    let mut out = Vec::with_capacity(count);
    for i in 0..count {
        let phase = (i % 320) as f32 / 319.0;
        let depth_factor = if phase < 0.20 {
            phase / 0.20
        } else if phase < 0.5 {
            1.0
        } else {
            (1.0 - phase) / 0.5
        };
        out.push(DiveMeasurement {
            depth: SURFACE + Pa::new(depth_factor * BOTTOM_DEPTH_DELTA.to_pa().to_f32()),
            time_ms: i * 1_000,
            gas: 0,
        });
    }
    out
}

#[test]
fn benchmark_small_profile() {
    let measurements = generate_profile_small(2_048);
    benchmark_dive_profile("profile.small", &measurements);
}

#[test]
fn benchmark_large_profile() {
    let measurements = generate_profile_large(2_048);
    benchmark_dive_profile("profile.large", &measurements);
}

#[test]
fn benchmark_deep_profile() {
    let measurements = generate_profile_deep(2 * 2_048);
    benchmark_dive_profile("profile.deep", &measurements);
}
