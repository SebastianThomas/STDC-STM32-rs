use stdc_diving_algorithms::{
    deco_algorithm::{DecoSettings, calc_deco_schedule},
    dive::{DiveMeasurement, get_ascent_rate_per_meter},
    gas::{
        AIR, GasDensitySettings, GasMix, MAX_GAS_DENSITY, NX50, NX100, TMX10_80, TissuesLoading,
    },
    o2tox::{O2ExposureType, O2ToxCalculation, O2ToxicityPercentage, calculate_toxicity_diff},
    pressure_unit::{Bar, Pa, Pressure, msw},
    setup::NUM_TISSUES,
};

use crate::{
    algorithms::profile_emulation::EmulatedDiveProfile,
    algorithms::rate_algorithm::{
        DecoUpdateRateAlgorithm, FlashLogRateAlgorithm, O2ToxUpdateRateAlgorithm, RateAlgorithm,
    },
    benchmarking::{self, BenchmarkSample},
    constants::barometric::SURFACE_PA,
    dive_log_host::{
        DecoAlgorithmType, LevelState, LogDiveControlDataBlock, LogPointData, LogPointMetadata,
    },
};

const DIVE_DISPLAY_REFRESH_INTERVAL_MILLIS: u32 = 250;

use std::sync::Mutex;
use std::vec::Vec;
#[cfg(all(test, not(target_os = "none")))]
use std::{
    fs::{self, File},
    io::{self, Write},
    path::PathBuf,
    time::{SystemTime, UNIX_EPOCH},
};

static REPORT_PRINT_LOCK: Mutex<()> = Mutex::new(());

struct SchedulerSet {
    deco_rate: DecoUpdateRateAlgorithm,
    o2_rate: O2ToxUpdateRateAlgorithm,
    flash_rate: FlashLogRateAlgorithm,
}

impl SchedulerSet {
    fn new() -> Self {
        SchedulerSet {
            deco_rate: DecoUpdateRateAlgorithm::default(),
            o2_rate: O2ToxUpdateRateAlgorithm::default(),
            flash_rate: FlashLogRateAlgorithm::default(),
        }
    }
}

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
    sum_cycles: u64,
    min_cycles: u64,
    median_cycles: u64,
    avg_cycles: u64,
    max_cycles: u64,
    sum_nanos: u64,
    min_nanos: u64,
    median_nanos: u64,
    avg_nanos: u64,
    max_nanos: u64,
}

#[derive(Clone, Copy)]
struct DecisionSample {
    due: bool,
    skipped_iterations: usize,
    skipped_seconds: f32,
}

#[derive(Clone, Copy)]
struct DecisionSampleStats {
    count: usize,
    due_count: usize,
    not_due_count: usize,
    sum_skipped_iterations: usize,
    min_skipped_iterations: usize,
    median_skipped_iterations: f32,
    avg_skipped_iterations: f32,
    max_skipped_iterations: usize,
    sum_skipped_seconds: f32,
    min_skipped_seconds: f32,
    median_skipped_seconds: f32,
    avg_skipped_seconds: f32,
    max_skipped_seconds: f32,
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
        sum_cycles: total_cycles as u64,
        min_cycles,
        median_cycles: median(&cycles),
        avg_cycles: (total_cycles / samples.len() as u128) as u64,
        max_cycles,
        sum_nanos: total_nanos as u64,
        min_nanos,
        median_nanos: median(&nanos),
        avg_nanos: (total_nanos / samples.len() as u128) as u64,
        max_nanos,
    }
}

fn stats_row(label: &str, stats: BenchmarkSampleStats) {
    print_row(&format!(
        "{:<24} | {:<6} | {:<12} | {:<12} | {:<12} | {:<12} | {:<12} | {:<12} | {:<12} | {:<12} | {:<12} | {:<12}",
        label,
        stats.count,
        format!("{} cyc", stats.sum_cycles),
        format!("{} cyc", stats.min_cycles),
        format!("{} cyc", stats.median_cycles),
        format!("{} cyc", stats.avg_cycles),
        format!("{} cyc", stats.max_cycles),
        format!("{} ns", stats.sum_nanos),
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
                sum_cycles: 0,
                min_cycles: 0,
                median_cycles: 0,
                avg_cycles: 0,
                max_cycles: 0,
                sum_nanos: 0,
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

fn summarize_decision_samples(samples: &[DecisionSample]) -> DecisionSampleStats {
    assert!(!samples.is_empty());

    let mut due_skipped_iterations: Vec<usize> = samples
        .iter()
        .filter(|sample| sample.due)
        .map(|sample| sample.skipped_iterations)
        .collect();
    let mut due_skipped_seconds: Vec<f32> = samples
        .iter()
        .filter(|sample| sample.due)
        .map(|sample| sample.skipped_seconds)
        .collect();
    due_skipped_iterations.sort_unstable();
    due_skipped_seconds.sort_by(|left, right| left.partial_cmp(right).unwrap());

    let due_count = due_skipped_iterations.len();
    let not_due_count = samples.len() - due_count;

    let median_usize = |values: &[usize]| -> f32 {
        if values.is_empty() {
            return 0.0;
        }
        if values.len() % 2 == 0 {
            let upper = values.len() / 2;
            let lower = upper - 1;
            (values[lower] as f32 + values[upper] as f32) / 2.0
        } else {
            values[values.len() / 2] as f32
        }
    };

    let median_f32 = |values: &[f32]| -> f32 {
        if values.is_empty() {
            return 0.0;
        }
        if values.len() % 2 == 0 {
            let upper = values.len() / 2;
            let lower = upper - 1;
            (values[lower] + values[upper]) / 2.0
        } else {
            values[values.len() / 2]
        }
    };

    let mut sum_skipped_iterations = 0usize;
    let mut sum_skipped_seconds = 0.0f32;
    let mut min_skipped_iterations = usize::MAX;
    let mut max_skipped_iterations = 0usize;
    let mut min_skipped_seconds = f32::INFINITY;
    let mut max_skipped_seconds = 0.0f32;
    for sample in samples.iter().filter(|sample| sample.due) {
        sum_skipped_iterations += sample.skipped_iterations;
        sum_skipped_seconds += sample.skipped_seconds;
        min_skipped_iterations = min_skipped_iterations.min(sample.skipped_iterations);
        max_skipped_iterations = max_skipped_iterations.max(sample.skipped_iterations);
        min_skipped_seconds = min_skipped_seconds.min(sample.skipped_seconds);
        max_skipped_seconds = max_skipped_seconds.max(sample.skipped_seconds);
    }

    if due_count == 0 {
        min_skipped_iterations = 0;
        min_skipped_seconds = 0.0;
    }

    DecisionSampleStats {
        count: samples.len(),
        due_count,
        not_due_count,
        sum_skipped_iterations,
        min_skipped_iterations,
        median_skipped_iterations: median_usize(&due_skipped_iterations),
        avg_skipped_iterations: if due_count == 0 {
            0.0
        } else {
            sum_skipped_iterations as f32 / due_count as f32
        },
        max_skipped_iterations,
        sum_skipped_seconds,
        min_skipped_seconds,
        median_skipped_seconds: median_f32(&due_skipped_seconds),
        avg_skipped_seconds: if due_count == 0 {
            0.0
        } else {
            sum_skipped_seconds / due_count as f32
        },
        max_skipped_seconds,
    }
}

fn print_decision_stats(label: &str, stats: DecisionSampleStats) {
    print_row(&format!(
        "{:<24} | {:<6} | {:<6} | {:<6} | {:<10} | {:<10} | {:<10} | {:<10} | {:<10} | {:<10} | {:<10} | {:<10} | {:<10} | {:<10}",
        label,
        stats.count,
        stats.due_count,
        stats.not_due_count,
        stats.sum_skipped_iterations,
        stats.min_skipped_iterations,
        format!("{:.1}", stats.median_skipped_iterations),
        format!("{:.1}", stats.avg_skipped_iterations),
        stats.max_skipped_iterations,
        format!("{:.3}", stats.sum_skipped_seconds),
        format!("{:.3}", stats.min_skipped_seconds),
        format!("{:.3}", stats.median_skipped_seconds),
        format!("{:.3}", stats.avg_skipped_seconds),
        format!("{:.3}", stats.max_skipped_seconds)
    ));
}

fn collect_decision_samples<State, IntervalFn, UpdateFn>(
    measurements: &[DiveMeasurement<Pa>],
    time_step_ms: u32,
    mut state: State,
    mut interval_fn: IntervalFn,
    mut update_fn: UpdateFn,
) -> Vec<DecisionSample>
where
    IntervalFn: FnMut(&State, &DiveMeasurement<Pa>, u32) -> u32,
    UpdateFn: FnMut(&mut State, &DiveMeasurement<Pa>),
{
    assert!(!measurements.is_empty());

    let mut samples: Vec<DecisionSample> = Vec::with_capacity(measurements.len());
    let mut last_update_millis = measurements[0].time_ms as u32;
    let mut last_update_index = 0usize;

    for (index, measurement) in measurements.iter().enumerate() {
        let elapsed_millis = (measurement.time_ms as u32).wrapping_sub(last_update_millis);
        let next_interval = interval_fn(&state, measurement, elapsed_millis);
        let due = elapsed_millis >= next_interval;
        let skipped_iterations = if due {
            index.saturating_sub(last_update_index + 1)
        } else {
            0
        };
        let skipped_seconds = if due {
            skipped_iterations as f32 * time_step_ms as f32 / 1_000.0
        } else {
            0.0
        };

        samples.push(DecisionSample {
            due,
            skipped_iterations,
            skipped_seconds,
        });

        if due {
            last_update_millis = measurement.time_ms as u32;
            last_update_index = index;
            update_fn(&mut state, measurement);
        }
    }

    samples
}

fn collect_deco_rate_decision_samples(
    measurements: &[DiveMeasurement<Pa>],
    time_step_ms: u32,
) -> Vec<DecisionSample> {
    collect_decision_samples(
        measurements,
        time_step_ms,
        measurements[0].depth.to_msw(),
        |previous_depth, measurement, elapsed_millis| {
            let mut rate_algorithm = DecoUpdateRateAlgorithm::default();
            rate_algorithm
                .next_iter(
                    (*previous_depth, elapsed_millis),
                    measurement.depth.to_msw(),
                )
                .unwrap()
        },
        |previous_depth, measurement| {
            *previous_depth = measurement.depth.to_msw();
        },
    )
}

fn collect_o2_rate_decision_samples(
    measurements: &[DiveMeasurement<Pa>],
    time_step_ms: u32,
) -> Vec<DecisionSample> {
    collect_decision_samples(
        measurements,
        time_step_ms,
        measurements[0].depth.to_msw(),
        |previous_depth, measurement, _| {
            let mut rate_algorithm = O2ToxUpdateRateAlgorithm::default();
            rate_algorithm
                .next_iter(*previous_depth, measurement.depth.to_msw())
                .unwrap()
        },
        |previous_depth, measurement| {
            *previous_depth = measurement.depth.to_msw();
        },
    )
}

fn collect_display_refresh_decision_samples(
    measurements: &[DiveMeasurement<Pa>],
    time_step_ms: u32,
) -> Vec<DecisionSample> {
    collect_decision_samples(
        measurements,
        time_step_ms,
        (),
        |_, _, _| DIVE_DISPLAY_REFRESH_INTERVAL_MILLIS,
        |_, _| {},
    )
}

fn collect_deco_rate_decision_samples_with(
    measurements: &[DiveMeasurement<Pa>],
    time_step_ms: u32,
    rate_algorithm: &mut DecoUpdateRateAlgorithm,
) -> Vec<DecisionSample> {
    collect_decision_samples(
        measurements,
        time_step_ms,
        measurements[0].depth.to_msw(),
        |previous_depth, measurement, elapsed_millis| {
            rate_algorithm
                .next_iter(
                    (*previous_depth, elapsed_millis),
                    measurement.depth.to_msw(),
                )
                .unwrap()
        },
        |previous_depth, measurement| {
            *previous_depth = measurement.depth.to_msw();
        },
    )
}

fn collect_o2_rate_decision_samples_with(
    measurements: &[DiveMeasurement<Pa>],
    time_step_ms: u32,
    rate_algorithm: &mut O2ToxUpdateRateAlgorithm,
) -> Vec<DecisionSample> {
    collect_decision_samples(
        measurements,
        time_step_ms,
        measurements[0].depth.to_msw(),
        |previous_depth, measurement, _| {
            rate_algorithm
                .next_iter(*previous_depth, measurement.depth.to_msw())
                .unwrap()
        },
        |previous_depth, measurement| {
            *previous_depth = measurement.depth.to_msw();
        },
    )
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
    let surface = SURFACE_PA;
    let mut out = Vec::with_capacity(count);
    for i in 0..count {
        out.push(DiveMeasurement {
            depth: profile.depth_at(surface, i),
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

    let max_depth_index = measurements
        .iter()
        .enumerate()
        .max_by(|(_, left), (_, right)| {
            left.depth
                .to_f32()
                .partial_cmp(&right.depth.to_f32())
                .unwrap()
        })
        .map(|(index, _)| index)
        .unwrap_or(0);

    let mut previous = measurements[0];
    for (i, measurement) in measurements.iter().enumerate().take(max_depth_index + 1) {
        let delta_ms = measurement.time_ms.saturating_sub(previous.time_ms);
        let delta_ms = if delta_ms > u16::MAX as usize {
            u16::MAX
        } else {
            delta_ms as u16
        };
        if i < 6 {
            eprintln!(
                "loading[{}]: delta_ms={}, depth_pa={:.0}",
                i,
                delta_ms,
                measurement.depth.to_f32()
            );
        }
        loading.tick(delta_ms, measurement.depth, &AIR);
        previous = *measurement;
    }

    let schedule =
        match calc_deco_schedule::<16, NUM_GASES>(&loading, gases, gases_enabled, deco_settings) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("calc_deco_schedule returned error: {}", e);
                return None;
            }
        };
    eprintln!("calc_deco_schedule returned schedule: {:?}", schedule);
    eprintln!("first_stop: {:?}", schedule.first_stop());
    // Debug: print a few tissue loadings and m-values to diagnose why no stop was selected
    use stdc_diving_algorithms::deco_algorithm::MVALUES;
    eprintln!(
        "loading.n2[0..4]: {:?}",
        [
            loading.n2[0].to_f32(),
            loading.n2[1].to_f32(),
            loading.n2[2].to_f32(),
            loading.n2[3].to_f32(),
        ]
    );
    eprintln!(
        "MVALUES[0].max_saturation[0..3]: {:?}",
        [
            MVALUES[0].max_saturation[0].to_f32(),
            MVALUES[0].max_saturation[1].to_f32(),
            MVALUES[0].max_saturation[2].to_f32(),
        ]
    );
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
    let gases = [AIR, NX100, NX50, TMX10_80];
    let gases_enabled = [true; 4];
    let deco_settings = DecoSettings {
        gas_density_settings: GasDensitySettings::Limit {
            limit_g_l: MAX_GAS_DENSITY,
        },
        max_deco_po2: Bar::new(1.6).to_pa(),
    };

    let base = generate_emulated_profile(count, profile, time_step_ms);
    let Some(overlay) =
        derive_deco_overlay::<4>(&base, &gases, &gases_enabled, &deco_settings, time_step_ms)
    else {
        return (profile, base);
    };

    let realized_profile =
        profile.with_deco_overlay(Pa::new(overlay.stop_depth_pa), overlay.hold_samples);
    let realized_measurements = generate_emulated_profile(count, realized_profile, time_step_ms);
    (realized_profile, realized_measurements)
}

#[cfg(all(test, not(target_os = "none")))]
fn export_benchmark_run(
    test_name: &str,
    profile: EmulatedDiveProfile,
    time_step_ms: u32,
    measurements: &[DiveMeasurement<Pa>],
    window_reports: &[(
        usize,
        Vec<BenchmarkSample>,
        Vec<BenchmarkSample>,
        Vec<BenchmarkSample>,
    )],
    rate_samples: &[BenchmarkSample],
    deco_rate_samples: &[BenchmarkSample],
    deco_rate_decision_samples: &[DecisionSample],
    display_refresh_rate_samples: &[BenchmarkSample],
    display_refresh_decision_samples: &[DecisionSample],
    o2_samples: &[BenchmarkSample],
    o2_rate_samples: &[BenchmarkSample],
    o2_rate_decision_samples: &[DecisionSample],
    control_samples: &[BenchmarkSample],
    point_samples: &[BenchmarkSample],
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
    writeln!(
        profile_csv,
        "# profile.csv records the realized emulated dive profile used for this benchmark run."
    )?;
    writeln!(
        profile_csv,
        "# sample_index is the zero-based sample number, time_ms is the simulated timestamp, stage is the dive phase, and depth_* is the measured depth at that sample."
    )?;
    writeln!(profile_csv, "sample_index,time_ms,stage,depth_pa,depth_msw")?;
    for (index, measurement) in measurements.iter().enumerate() {
        let point = profile.point_at(SURFACE_PA, time_step_ms, index);
        writeln!(
            profile_csv,
            "{},{},{:?},{:.2},{:.4}",
            point.sample_index,
            point.time_ms,
            point.stage,
            measurement.depth.to_f32(),
            point.depth_msw.to_f32()
        )?;
    }

    let mut summary_csv = File::create(base_dir.join("summary.csv"))?;
    writeln!(
        summary_csv,
        "# summary.csv aggregates benchmark timing samples for the run."
    )?;
    writeln!(
        summary_csv,
        "# section identifies the kind of record: window for per-window batches or single for one-off measurements."
    )?;
    writeln!(
        summary_csv,
        "# label names the benchmarked operation or sub-metric within that section."
    )?;
    writeln!(
        summary_csv,
        "# window labels: loading.window, deco.window.<window_size>, flash.log.rate.window.<window_size>. single labels: flash.log.rate, deco.schedule.rate, o2.tox, o2.tox.rate, log.control.serialize, log.point.serialize."
    )?;
    writeln!(
        summary_csv,
        "# loading.window and rate.* are the executed runtime algorithms, deco.window.* is the deco scheduling calculation, deco.schedule.rate and o2.tox.rate are the dynamic interval algorithms, and o2.tox / log.* are support calculations."
    )?;
    writeln!(
        summary_csv,
        "# the single rows summarize per-invocation measurements for each replayed call, so their count, sum, min, median, avg, and max reflect the underlying distribution."
    )?;
    writeln!(
        summary_csv,
        "# the decision rows track whether a scheduler triggered a run or skipped it, plus how many replay iterations and seconds were skipped before each run."
    )?;
    writeln!(
        summary_csv,
        "section,label,count,sum_cycles,min_cycles,median_cycles,avg_cycles,max_cycles,sum_ns,min_ns,median_ns,avg_ns,max_ns"
    )?;

    for (window_size, loading_samples, deco_samples, rate_samples) in window_reports {
        let loading_stats = summarize_samples(loading_samples);
        let deco_stats = summarize_samples(deco_samples);
        let rate_stats = summarize_samples(rate_samples);
        writeln!(
            summary_csv,
            "window,{},{},{},{},{},{},{},{},{},{},{},{}",
            window_size,
            loading_stats.count,
            loading_stats.sum_cycles,
            loading_stats.min_cycles,
            loading_stats.median_cycles,
            loading_stats.avg_cycles,
            loading_stats.max_cycles,
            loading_stats.sum_nanos,
            loading_stats.min_nanos,
            loading_stats.median_nanos,
            loading_stats.avg_nanos,
            loading_stats.max_nanos,
        )?;
        writeln!(
            summary_csv,
            "window,{},{},{},{},{},{},{},{},{},{},{},{}",
            format!("deco.window.{window_size}"),
            deco_stats.count,
            deco_stats.sum_cycles,
            deco_stats.min_cycles,
            deco_stats.median_cycles,
            deco_stats.avg_cycles,
            deco_stats.max_cycles,
            deco_stats.sum_nanos,
            deco_stats.min_nanos,
            deco_stats.median_nanos,
            deco_stats.avg_nanos,
            deco_stats.max_nanos,
        )?;
        writeln!(
            summary_csv,
            "window,{},{},{},{},{},{},{},{},{},{},{},{}",
            format!("rate.window.{window_size}"),
            rate_stats.count,
            rate_stats.sum_cycles,
            rate_stats.min_cycles,
            rate_stats.median_cycles,
            rate_stats.avg_cycles,
            rate_stats.max_cycles,
            rate_stats.sum_nanos,
            rate_stats.min_nanos,
            rate_stats.median_nanos,
            rate_stats.avg_nanos,
            rate_stats.max_nanos,
        )?;
    }

    // single measurements: explicit to include display.refresh.rate
    let stats = summarize_samples(rate_samples);
    writeln!(
        summary_csv,
        "single,rate.algorithm,{},{},{},{},{},{},{},{},{},{},{}",
        stats.count,
        stats.sum_cycles,
        stats.min_cycles,
        stats.median_cycles,
        stats.avg_cycles,
        stats.max_cycles,
        stats.sum_nanos,
        stats.min_nanos,
        stats.median_nanos,
        stats.avg_nanos,
        stats.max_nanos
    )?;
    let stats = summarize_samples(deco_rate_samples);
    writeln!(
        summary_csv,
        "single,deco.schedule.rate,{},{},{},{},{},{},{},{},{},{},{}",
        stats.count,
        stats.sum_cycles,
        stats.min_cycles,
        stats.median_cycles,
        stats.avg_cycles,
        stats.max_cycles,
        stats.sum_nanos,
        stats.min_nanos,
        stats.median_nanos,
        stats.avg_nanos,
        stats.max_nanos
    )?;
    let stats = summarize_samples(o2_samples);
    writeln!(
        summary_csv,
        "single,o2.tox,{},{},{},{},{},{},{},{},{},{},{}",
        stats.count,
        stats.sum_cycles,
        stats.min_cycles,
        stats.median_cycles,
        stats.avg_cycles,
        stats.max_cycles,
        stats.sum_nanos,
        stats.min_nanos,
        stats.median_nanos,
        stats.avg_nanos,
        stats.max_nanos
    )?;
    let stats = summarize_samples(o2_rate_samples);
    writeln!(
        summary_csv,
        "single,o2.tox.rate,{},{},{},{},{},{},{},{},{},{},{}",
        stats.count,
        stats.sum_cycles,
        stats.min_cycles,
        stats.median_cycles,
        stats.avg_cycles,
        stats.max_cycles,
        stats.sum_nanos,
        stats.min_nanos,
        stats.median_nanos,
        stats.avg_nanos,
        stats.max_nanos
    )?;
    let stats = summarize_samples(display_refresh_rate_samples);
    writeln!(
        summary_csv,
        "single,display.refresh.rate,{},{},{},{},{},{},{},{},{},{},{}",
        stats.count,
        stats.sum_cycles,
        stats.min_cycles,
        stats.median_cycles,
        stats.avg_cycles,
        stats.max_cycles,
        stats.sum_nanos,
        stats.min_nanos,
        stats.median_nanos,
        stats.avg_nanos,
        stats.max_nanos
    )?;
    let stats = summarize_samples(control_samples);
    writeln!(
        summary_csv,
        "single,log.control.serialize,{},{},{},{},{},{},{},{},{},{},{}",
        stats.count,
        stats.sum_cycles,
        stats.min_cycles,
        stats.median_cycles,
        stats.avg_cycles,
        stats.max_cycles,
        stats.sum_nanos,
        stats.min_nanos,
        stats.median_nanos,
        stats.avg_nanos,
        stats.max_nanos
    )?;
    let stats = summarize_samples(point_samples);
    writeln!(
        summary_csv,
        "single,log.point.serialize,{},{},{},{},{},{},{},{},{},{},{}",
        stats.count,
        stats.sum_cycles,
        stats.min_cycles,
        stats.median_cycles,
        stats.avg_cycles,
        stats.max_cycles,
        stats.sum_nanos,
        stats.min_nanos,
        stats.median_nanos,
        stats.avg_nanos,
        stats.max_nanos
    )?;

    let mut manifest = File::create(base_dir.join("manifest.txt"))?;
    writeln!(manifest, "test_name={}", test_name)?;
    writeln!(manifest, "profile_cycle_length={}", profile.cycle_length())?;
    writeln!(manifest, "measurements={}", measurements.len())?;
    writeln!(manifest, "time_step_ms={}", time_step_ms)?;

    let mut decision_csv = File::create(base_dir.join("decision.csv"))?;
    writeln!(
        decision_csv,
        "# decision.csv summarizes scheduler decisions for the replayed benchmark run."
    )?;
    writeln!(
        decision_csv,
        "# label names the scheduler and the counts/intervals describe due versus skipped replay iterations."
    )?;
    writeln!(
        decision_csv,
        "label,count,due_count,not_due_count,sum_skipped_iterations,min_skipped_iterations,median_skipped_iterations,avg_skipped_iterations,max_skipped_iterations,sum_skipped_seconds,min_skipped_seconds,median_skipped_seconds,avg_skipped_seconds,max_skipped_seconds"
    )?;
    let decision_labels = ["deco.schedule.rate", "o2.tox.rate", "display.refresh.rate"];
    let decision_samples = [
        deco_rate_decision_samples,
        o2_rate_decision_samples,
        display_refresh_decision_samples,
    ];
    for (label, samples) in decision_labels.iter().zip(decision_samples) {
        let stats = summarize_decision_samples(samples);
        writeln!(
            decision_csv,
            "{},{},{},{},{},{},{:.1},{:.1},{},{:.3},{:.3},{:.3},{:.3},{:.3}",
            label,
            stats.count,
            stats.due_count,
            stats.not_due_count,
            stats.sum_skipped_iterations,
            stats.min_skipped_iterations,
            stats.median_skipped_iterations,
            stats.avg_skipped_iterations,
            stats.max_skipped_iterations,
            stats.sum_skipped_seconds,
            stats.min_skipped_seconds,
            stats.median_skipped_seconds,
            stats.avg_skipped_seconds,
            stats.max_skipped_seconds,
        )?;
    }

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
    let gases = [AIR, NX50, NX100, TMX10_80];
    let gases_enabled = [true; 4];
    let deco_settings = DecoSettings {
        gas_density_settings: GasDensitySettings::Limit {
            limit_g_l: MAX_GAS_DENSITY,
        },
        max_deco_po2: Bar::new(1.6).to_pa(),
    };

    let mut sched = SchedulerSet::new();

    let mut window_reports: Vec<(
        usize,
        Vec<BenchmarkSample>,
        Vec<BenchmarkSample>,
        Vec<BenchmarkSample>,
    )> = Vec::new();
    for window_size in WINDOW_SIZES {
        let mut loading = TissuesLoading::<NUM_TISSUES, Pa>::new(surface, &AIR);
        let mut window_rate = FlashLogRateAlgorithm::default();
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
                calc_deco_schedule::<16, 4>(&loading, &gases, &gases_enabled, &deco_settings)
            });
            deco_samples.push(deco_sample);

            let ((), rate_window_sample) = benchmarking::measure("flash.log.rate.window", || {
                let mut last_rate_time = measurements[window_start].time_ms;
                for measurement in &measurements[window_start..window_end] {
                    let elapsed_ms = measurement.time_ms.saturating_sub(last_rate_time) as u32;
                    let _ =
                        window_rate.next_iter((current_for_rate, elapsed_ms), measurement.depth);
                    current_for_rate = measurement.depth;
                    last_rate_time = measurement.time_ms;
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

    let mut rate_algorithm_samples: Vec<BenchmarkSample> = Vec::new();
    let mut previous = measurements[0];
    for measurement in measurements {
        let elapsed_ms = measurement
            .time_ms
            .saturating_sub(previous.time_ms)
            .try_into()
            .unwrap_or(u32::MAX);
        let (_, sample) = benchmarking::measure("flash.log.rate", || {
            sched
                .flash_rate
                .next_iter((previous.depth, elapsed_ms), measurement.depth)
        });
        rate_algorithm_samples.push(sample);
        previous = *measurement;
    }

    let mut deco_rate_samples: Vec<BenchmarkSample> = Vec::new();
    let mut previous = measurements[0];
    for measurement in measurements {
        let delta_ms = measurement.time_ms.saturating_sub(previous.time_ms);
        let delta_ms = if delta_ms > u16::MAX as usize {
            u16::MAX
        } else {
            delta_ms as u16
        };
        let (_, sample) = benchmarking::measure("deco.schedule.rate", || {
            sched
                .deco_rate
                .next_iter(
                    (previous.depth.to_msw(), delta_ms as u32),
                    measurement.depth.to_msw(),
                )
                .unwrap()
        });
        deco_rate_samples.push(sample);
        previous = *measurement;
    }

    let mut o2_tox_samples: Vec<BenchmarkSample> = Vec::new();
    let mut o2_rate_samples: Vec<BenchmarkSample> = Vec::new();
    for measurement in measurements {
        let (_, tox_sample) = benchmarking::measure("o2.tox", || {
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
        o2_tox_samples.push(tox_sample);

        let (_, rate_sample) = benchmarking::measure("o2.tox.rate", || {
            sched
                .o2_rate
                .next_iter(measurement.depth.to_msw(), measurement.depth.to_msw())
                .unwrap()
        });
        o2_rate_samples.push(rate_sample);
    }

    let display_refresh_rate_samples: Vec<BenchmarkSample> = measurements
        .iter()
        .map(|_| {
            benchmarking::measure("display.refresh.rate", || {
                DIVE_DISPLAY_REFRESH_INTERVAL_MILLIS
            })
            .1
        })
        .collect();

    let deco_rate_decision_samples =
        collect_deco_rate_decision_samples_with(measurements, time_step_ms, &mut sched.deco_rate);
    let o2_rate_decision_samples =
        collect_o2_rate_decision_samples_with(measurements, time_step_ms, &mut sched.o2_rate);
    let display_refresh_decision_samples =
        collect_display_refresh_decision_samples(measurements, time_step_ms);

    let control_block = LogDiveControlDataBlock::<4>::new(
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
            &rate_algorithm_samples,
            &deco_rate_samples,
            &deco_rate_decision_samples,
            &display_refresh_rate_samples,
            &display_refresh_decision_samples,
            &o2_tox_samples,
            &o2_rate_samples,
            &o2_rate_decision_samples,
            std::slice::from_ref(&control_sample),
            std::slice::from_ref(&point_sample),
        )
        .expect("export benchmark artifacts");
        assert!(export_dir.join("profile.csv").exists());
        assert!(export_dir.join("summary.csv").exists());
        assert!(export_dir.join("decision.csv").exists());
        assert!(export_dir.join("manifest.txt").exists());
    }

    let _guard = REPORT_PRINT_LOCK.lock().unwrap();
    println!();
    print_title(&format!("Benchmark Report - {}", name));
    for (window_size, loading_samples, deco_samples, rate_samples) in &window_reports {
        print_row(&format!("window_size={} measurements={}", window_size, n));
        print_rule();
        print_row(&format!(
            "{:<24} | {:<6} | {:<12} | {:<12} | {:<12} | {:<12} | {:<12} | {:<12} | {:<12} | {:<12} | {:<12} | {:<12}",
            "window metric",
            "count",
            "sum",
            "min",
            "median",
            "avg",
            "max",
            "sum",
            "min",
            "median",
            "avg",
            "max"
        ));
        print_rule();
        print_sample_stats("loading.window", loading_samples);
        print_sample_stats("deco.schedule.window", deco_samples);
        print_sample_stats("flash.log.rate.window", rate_samples);
        print_rule();
    }
    print_row(&format!(
        "{:<24} | {:<6} | {:<12} | {:<12} | {:<12} | {:<12} | {:<12} | {:<12} | {:<12} | {:<12} | {:<12} | {:<12}",
        "single metric",
        "count",
        "sum",
        "min",
        "median",
        "avg",
        "max",
        "sum",
        "min",
        "median",
        "avg",
        "max"
    ));
    print_rule();
    print_sample_stats("flash.log.rate", &rate_algorithm_samples);
    print_sample_stats("deco.schedule.rate", &deco_rate_samples);
    print_sample_stats("o2.tox", &o2_tox_samples);
    print_sample_stats("o2.tox.rate", &o2_rate_samples);
    print_sample_stats("display.refresh.rate", &display_refresh_rate_samples);
    print_sample_stats(
        "log.control.serialize",
        std::slice::from_ref(&control_sample),
    );
    print_sample_stats("log.point.serialize", std::slice::from_ref(&point_sample));
    print_rule();
    print_row(&format!(
        "{:<24} | {:<6} | {:<6} | {:<6} | {:<10} | {:<10} | {:<10} | {:<10} | {:<10} | {:<10} | {:<10} | {:<10} | {:<10} | {:<10}",
        "decision metric",
        "count",
        "due",
        "skip",
        "sum_it",
        "min_it",
        "med_it",
        "avg_it",
        "max_it",
        "sum_s",
        "min_s",
        "med_s",
        "avg_s",
        "max_s"
    ));
    print_rule();
    print_decision_stats(
        "deco.schedule.rate",
        summarize_decision_samples(&deco_rate_decision_samples),
    );
    print_decision_stats(
        "o2.tox.rate",
        summarize_decision_samples(&o2_rate_decision_samples),
    );
    print_decision_stats(
        "display.refresh.rate",
        summarize_decision_samples(&display_refresh_decision_samples),
    );
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

fn generate_profile_deep_long(count: usize) -> (EmulatedDiveProfile, Vec<DiveMeasurement<Pa>>) {
    let profile = EmulatedDiveProfile::new([40, 3000, 160], msw::new(90.0).to_pa(), 0.02);
    generate_deco_aware_profile(count, profile, 200)
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

#[test]
fn benchmark_deep_long_profile() {
    let (profile, measurements) = generate_profile_deep_long(2 * 4_096);
    benchmark_dive_profile("profile.deep.long", profile, 200, &measurements);
}
