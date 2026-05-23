use stdc_diving_algorithms::{
    deco_algorithm::{DecoSettings, calc_deco_schedule},
    dive::{DiveMeasurement, get_ascent_rate_per_meter},
    gas::{AIR, GasDensitySettings, GasMix, MAX_GAS_DENSITY, TissuesLoading},
    o2tox::{O2ExposureType, O2ToxCalculation, O2ToxicityPercentage, calculate_toxicity_diff},
    pressure_unit::{Bar, Pa, Pressure},
    setup::NUM_TISSUES,
};

use crate::{
    algorithms::rate_algorithm::{DynamicDiffAimdRateAlgorithm, RateAlgorithm},
    algorithms::profile_emulation::EmulatedDiveProfile,
    benchmarking::{self, BenchmarkSample},
    dive_log_host::{
        DecoAlgorithmType, LevelState, LogDiveControlDataBlock, LogPointData, LogPointMetadata,
    },
};

#[cfg(all(test, not(target_os = "none")))]
use std::{
    fs::{self, File},
    io::{self, Write},
    path::PathBuf,
    time::{SystemTime, UNIX_EPOCH},
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

#[derive(Clone, Copy)]
struct BenchmarkSampleStats {
    count: usize,
    min_cycles: u64,
    median_cycles: u64,
    avg_cycles: u64,
    max_cycles: u64,
    min_nanos: u64,
    median_nanos: u64,
    avg_nanos: u64,
    max_nanos: u64,
}

fn summarize_samples(samples: &[BenchmarkSample]) -> BenchmarkSampleStats {
    assert!(!samples.is_empty());

    let mut cycles: Vec<u64> = samples.iter().map(|sample| sample.cycles).collect();
    let mut nanos: Vec<u64> = samples.iter().map(|sample| sample.nanos).collect();
    cycles.sort_unstable();
    nanos.sort_unstable();

    let mut min_cycles = u64::MAX;
    let mut max_cycles = 0u64;
    let mut min_nanos = u64::MAX;
    let mut max_nanos = 0u64;
    let mut total_cycles: u128 = 0;
    let mut total_nanos: u128 = 0;
    for sample in samples {
        min_cycles = min_cycles.min(sample.cycles);
        max_cycles = max_cycles.max(sample.cycles);
        min_nanos = min_nanos.min(sample.nanos);
        max_nanos = max_nanos.max(sample.nanos);
        total_cycles += sample.cycles as u128;
        total_nanos += sample.nanos as u128;
    }

    let median = |values: &[u64]| -> u64 {
        if values.len() % 2 == 0 {
            let upper = values.len() / 2;
            let lower = upper - 1;
            (values[lower] as u128 + values[upper] as u128).div_euclid(2) as u64
        } else {
            values[values.len() / 2]
        }
    };

    BenchmarkSampleStats {
        count: samples.len(),
        min_cycles,
        median_cycles: median(&cycles),
        avg_cycles: (total_cycles / samples.len() as u128) as u64,
        max_cycles,
        min_nanos,
        median_nanos: median(&nanos),
        avg_nanos: (total_nanos / samples.len() as u128) as u64,
        max_nanos,
    }
}

fn stats_row(
    label: &str,
    stats: BenchmarkSampleStats,
) {
    print_row(&format!(
        "{:<24} | {:<6} | {:<12} | {:<12} | {:<12} | {:<12} | {:<12} | {:<12} | {:<12} | {:<12}",
        label,
        stats.count,
        format!("{} cyc", stats.min_cycles),
        format!("{} cyc", stats.median_cycles),
        format!("{} cyc", stats.avg_cycles),
        format!("{} cyc", stats.max_cycles),
        format!("{} ns", stats.min_nanos),
        format!("{} ns", stats.median_nanos),
        format!("{} ns", stats.avg_nanos),
        format!("{} ns", stats.max_nanos)
    ));
}

fn print_sample_stats(label: &str, samples: &[BenchmarkSample]) {
    if samples.is_empty() {
        stats_row(
            label,
            BenchmarkSampleStats {
                count: 0,
                min_cycles: 0,
                median_cycles: 0,
                avg_cycles: 0,
                max_cycles: 0,
                min_nanos: 0,
                median_nanos: 0,
                avg_nanos: 0,
                max_nanos: 0,
            },
        );
        return;
    }

    stats_row(label, summarize_samples(samples));
}

fn print_single_sample(sample: &BenchmarkSample) {
    print_row(&format!(
        "{:<24} | {:<14} | {:<14}",
        sample.label,
        format!("{} cyc", sample.cycles),
        format!("{} ns", sample.nanos)
    ));
}

const WINDOW_SIZES: [usize; 5] = [1, 2, 4, 8, 16];
const MIN_WINDOW_SAMPLE_COUNT: usize = 30;

#[derive(Clone, Copy)]
struct DecoOverlayInfo {
    stop_depth_pa: f32,
    hold_samples: usize,
}

fn generate_emulated_profile(
    count: usize,
    profile: EmulatedDiveProfile,
    time_step_ms: u32,
) -> Vec<DiveMeasurement<Pa>> {
    let surface = 101_325.0;
    let mut out = Vec::with_capacity(count);
    for i in 0..count {
        out.push(DiveMeasurement {
            depth: Pa::new(profile.depth_at(surface, i)),
            time_ms: i * time_step_ms as usize,
            gas: 0,
        });
    }
    out
}

fn derive_deco_overlay<const NUM_GASES: usize>(
    measurements: &[DiveMeasurement<Pa>],
    gases: &[GasMix<f32>; NUM_GASES],
    gases_enabled: &[bool; NUM_GASES],
    deco_settings: &DecoSettings<Pa>,
    time_step_ms: u32,
) -> Option<DecoOverlayInfo>
where
    [(); NUM_GASES * 3]: Sized,
    [(); 24 + NUM_GASES * 3]: Sized,
    [(); 4 + 24 + NUM_GASES * 3]: Sized,
    [(); 4 + (24 + NUM_GASES * 3)]: Sized,
    [(); 4 + (24 + NUM_GASES * 3) + 0]: Sized,
    [(); 24 + NUM_GASES * 3 + 0]: Sized,
{
    if measurements.is_empty() {
        return None;
    }

    let surface = Pa::new(101_325.0);
    let mut loading = TissuesLoading::<NUM_TISSUES, Pa>::new(surface, &AIR);
    let mut previous = measurements[0];

    for measurement in measurements {
        let delta_ms = measurement.time_ms.saturating_sub(previous.time_ms);
        let delta_ms = if delta_ms > u16::MAX as usize {
            u16::MAX
        } else {
            delta_ms as u16
        };
        loading.tick(delta_ms, measurement.depth, &AIR);
        previous = *measurement;
    }

    let schedule = calc_deco_schedule::<16, NUM_GASES>(&loading, gases, gases_enabled, deco_settings).ok()?;
    let first_stop = schedule.first_stop()?;
    let hold_time = schedule.get_deco_tts(&get_ascent_rate_per_meter(9));
    let hold_samples = hold_time
        .as_millis()
        .saturating_add(time_step_ms as u128 - 1)
        .div_euclid(time_step_ms as u128) as usize;

    Some(DecoOverlayInfo {
        stop_depth_pa: first_stop.depth().to_pa().to_f32(),
        hold_samples: hold_samples.max(1),
    })
}

fn generate_deco_aware_profile(
    count: usize,
    profile: EmulatedDiveProfile,
    time_step_ms: u32,
) -> (EmulatedDiveProfile, Vec<DiveMeasurement<Pa>>) {
    let gases = [AIR];
    let gases_enabled = [true; 1];
    let deco_settings = DecoSettings {
        gas_density_settings: GasDensitySettings::Limit {
            limit: MAX_GAS_DENSITY.to_pa(),
        },
        max_deco_po2: Bar::new(1.6).to_pa(),
    };

    let base = generate_emulated_profile(count, profile, time_step_ms);
    let Some(overlay) = derive_deco_overlay::<1>(
        &base,
        &gases,
        &gases_enabled,
        &deco_settings,
        time_step_ms,
    ) else {
        return (profile, base);
    };

    let realized_profile = profile.with_deco_overlay(overlay.stop_depth_pa, overlay.hold_samples);
    let realized_measurements = generate_emulated_profile(count, realized_profile, time_step_ms);
    (realized_profile, realized_measurements)
}

#[cfg(all(test, not(target_os = "none")))]
fn export_benchmark_run(
    test_name: &str,
    profile: EmulatedDiveProfile,
    time_step_ms: u32,
    measurements: &[DiveMeasurement<Pa>],
    window_reports: &[(usize, Vec<BenchmarkSample>, Vec<BenchmarkSample>, Vec<BenchmarkSample>)],
    rate_sample: &BenchmarkSample,
    o2_sample: &BenchmarkSample,
    control_sample: &BenchmarkSample,
    point_sample: &BenchmarkSample,
) -> io::Result<PathBuf> {
    let run_id = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    let base_dir = PathBuf::from("target/benchmark-results")
        .join(test_name)
        .join(run_id.to_string());
    fs::create_dir_all(&base_dir)?;

    let mut profile_csv = File::create(base_dir.join("profile.csv"))?;
    writeln!(profile_csv, "# profile.csv records the realized emulated dive profile used for this benchmark run.")?;
    writeln!(profile_csv, "# sample_index is the zero-based sample number, time_ms is the simulated timestamp, stage is the dive phase, and depth_* is the measured depth at that sample.")?;
    writeln!(profile_csv, "sample_index,time_ms,stage,depth_pa,depth_msw")?;
    for (index, measurement) in measurements.iter().enumerate() {
        let point = profile.point_at(101_325.0, time_step_ms, index);
        writeln!(
            profile_csv,
            "{},{},{:?},{:.2},{:.4}",
            point.sample_index,
            point.time_ms,
            point.stage,
            measurement.depth.to_f32(),
            point.depth_msw
        )?;
    }

    let mut summary_csv = File::create(base_dir.join("summary.csv"))?;
    writeln!(summary_csv, "# summary.csv aggregates benchmark timing samples for the run.")?;
    writeln!(summary_csv, "# section identifies the kind of record: window for per-window batches or single for one-off measurements.")?;
    writeln!(summary_csv, "# label names the benchmarked operation or sub-metric within that section.")?;
    writeln!(summary_csv, "# window labels: loading.window, deco.window.<window_size>, rate.window.<window_size>. single labels: rate.algorithm, o2.tox, log.control.serialize, log.point.serialize.")?;
    writeln!(summary_csv, "# loading.window and rate.* are the executed runtime algorithms, deco.window.* is the deco scheduling calculation, and o2.tox / log.* are support calculations.")?;
    writeln!(summary_csv, "# total is an overall aggregate across every timing sample written for the run.")?;
    writeln!(
        summary_csv,
        "section,label,count,min_cycles,median_cycles,avg_cycles,max_cycles,min_ns,median_ns,avg_ns,max_ns"
    )?;

    let mut total_samples: Vec<BenchmarkSample> = Vec::new();

    for (window_size, loading_samples, deco_samples, rate_samples) in window_reports {
        let loading_stats = summarize_samples(loading_samples);
        let deco_stats = summarize_samples(deco_samples);
        let rate_stats = summarize_samples(rate_samples);
        total_samples.extend_from_slice(loading_samples);
        total_samples.extend_from_slice(deco_samples);
        total_samples.extend_from_slice(rate_samples);
        writeln!(
            summary_csv,
            "window,{},{},{},{},{},{},{},{},{},{}",
            window_size,
            loading_stats.count,
            loading_stats.min_cycles,
            loading_stats.median_cycles,
            loading_stats.avg_cycles,
            loading_stats.max_cycles,
            loading_stats.min_nanos,
            loading_stats.median_nanos,
            loading_stats.avg_nanos,
            loading_stats.max_nanos,
        )?;
        writeln!(
            summary_csv,
            "window,{},{},{},{},{},{},{},{},{},{}",
            format!("deco.window.{window_size}"),
            deco_stats.count,
            deco_stats.min_cycles,
            deco_stats.median_cycles,
            deco_stats.avg_cycles,
            deco_stats.max_cycles,
            deco_stats.min_nanos,
            deco_stats.median_nanos,
            deco_stats.avg_nanos,
            deco_stats.max_nanos,
        )?;
        writeln!(
            summary_csv,
            "window,{},{},{},{},{},{},{},{},{},{}",
            format!("rate.window.{window_size}"),
            rate_stats.count,
            rate_stats.min_cycles,
            rate_stats.median_cycles,
            rate_stats.avg_cycles,
            rate_stats.max_cycles,
            rate_stats.min_nanos,
            rate_stats.median_nanos,
            rate_stats.avg_nanos,
            rate_stats.max_nanos,
        )?;
    }

    let single_samples = [rate_sample, o2_sample, control_sample, point_sample];
    let single_labels = [
        "rate.algorithm",
        "o2.tox",
        "log.control.serialize",
        "log.point.serialize",
    ];
    for (label, sample) in single_labels.iter().zip(single_samples) {
        total_samples.push(*sample);
        writeln!(
            summary_csv,
            "single,{},{},{},{},{},{},{},{},{},{}",
            label,
            1,
            sample.cycles,
            sample.cycles,
            sample.cycles,
            sample.cycles,
            sample.nanos,
            sample.nanos,
            sample.nanos,
            sample.nanos,
        )?;
    }

    let total_stats = summarize_samples(&total_samples);
    writeln!(
        summary_csv,
        "total,all,{},{},{},{},{},{},{},{},{}",
        total_stats.count,
        total_stats.min_cycles,
        total_stats.median_cycles,
        total_stats.avg_cycles,
        total_stats.max_cycles,
        total_stats.min_nanos,
        total_stats.median_nanos,
        total_stats.avg_nanos,
        total_stats.max_nanos,
    )?;

    let mut manifest = File::create(base_dir.join("manifest.txt"))?;
    writeln!(manifest, "test_name={}", test_name)?;
    writeln!(manifest, "profile_cycle_length={}", profile.cycle_length())?;
    writeln!(manifest, "measurements={}", measurements.len())?;
    writeln!(manifest, "time_step_ms={}", time_step_ms)?;

    Ok(base_dir)
}

fn benchmark_dive_profile(
    name: &'static str,
    profile: EmulatedDiveProfile,
    time_step_ms: u32,
    measurements: &[DiveMeasurement<Pa>],
) {
    assert!(!measurements.is_empty());
    assert!(
        measurements.len() >= MIN_WINDOW_SAMPLE_COUNT,
        "need at least {} measurements for windowed benchmarking",
        MIN_WINDOW_SAMPLE_COUNT
    );
    let n = measurements.len();

    let surface = Pa::new(101_325.0);
    let gases = [AIR];
    let gases_enabled = [true; 1];
    let deco_settings = DecoSettings {
        gas_density_settings: GasDensitySettings::Limit {
            limit: MAX_GAS_DENSITY.to_pa(),
        },
        max_deco_po2: Bar::new(1.6).to_pa(),
    };

    let mut window_reports: Vec<(usize, Vec<BenchmarkSample>, Vec<BenchmarkSample>, Vec<BenchmarkSample>)> = Vec::new();
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
            loading_samples.push(loading_sample);

            let (_, deco_sample) = benchmarking::measure("deco.schedule.window", || {
                calc_deco_schedule::<16, 1>(&loading, &gases, &gases_enabled, &deco_settings)
            });
            deco_samples.push(deco_sample);

            let ((), rate_window_sample) = benchmarking::measure("rate.algorithm.window", || {
                for measurement in &measurements[window_start..window_end] {
                    let _ = window_rate.next_iter(current_for_rate, measurement.depth);
                    current_for_rate = measurement.depth;
                }
            });
            rate_samples.push(rate_window_sample);

            window_start = window_end;
        }

        assert_eq!(
            loading_samples.len(),
            (n + window_size - 1) / window_size,
            "unexpected loading sample count for window_size={}",
            window_size
        );
        assert_eq!(
            deco_samples.len(),
            loading_samples.len(),
            "deco samples should match loading samples for window_size={}",
            window_size
        );
        assert_eq!(
            rate_samples.len(),
            loading_samples.len(),
            "rate samples should match loading samples for window_size={}",
            window_size
        );

        window_reports.push((window_size, loading_samples, deco_samples, rate_samples));
    }

    assert_eq!(window_reports.len(), WINDOW_SIZES.len());

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

    #[cfg(all(test, not(target_os = "none")))]
    {
        let export_dir = export_benchmark_run(
            name,
            profile,
            time_step_ms,
            measurements,
            &window_reports,
            &rate_sample,
            &o2_sample,
            &control_sample,
            &point_sample,
        )
        .expect("export benchmark artifacts");
        assert!(export_dir.join("profile.csv").exists());
        assert!(export_dir.join("summary.csv").exists());
        assert!(export_dir.join("manifest.txt").exists());
    }

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

fn generate_profile_small(count: usize) -> (EmulatedDiveProfile, Vec<DiveMeasurement<Pa>>) {
    generate_deco_aware_profile(count, EmulatedDiveProfile::small(), 15_000)
}

fn generate_profile_large(count: usize) -> (EmulatedDiveProfile, Vec<DiveMeasurement<Pa>>) {
    generate_deco_aware_profile(count, EmulatedDiveProfile::large(), 20_000)
}

fn generate_profile_deep(count: usize) -> (EmulatedDiveProfile, Vec<DiveMeasurement<Pa>>) {
    generate_deco_aware_profile(count, EmulatedDiveProfile::deep(), 1_000)
}

#[test]
fn benchmark_small_profile() {
    let (profile, measurements) = generate_profile_small(2_048);
    benchmark_dive_profile("profile.small", profile, 15_000, &measurements);
}

#[test]
fn benchmark_large_profile() {
    let (profile, measurements) = generate_profile_large(2_048);
    benchmark_dive_profile("profile.large", profile, 20_000, &measurements);
}

#[test]
fn benchmark_deep_profile() {
    let (profile, measurements) = generate_profile_deep(2 * 2_048);
    benchmark_dive_profile("profile.deep", profile, 1_000, &measurements);
}
