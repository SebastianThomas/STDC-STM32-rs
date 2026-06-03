use core::time::Duration;
use num::ToPrimitive;
use rtt_target::rprintln;
use stdc_diving_algorithms::{
    deco_algorithm::{DecoSettings, MVALUES, TISSUES, update_model_state},
    dive::DiveMeasurement,
    gas::{GasDensitySettings, GasMix, MAX_GAS_DENSITY},
    o2tox::{O2ExposureType, O2ToxCalculation, calculate_toxicity_diff},
    pressure_unit::{Bar, Pa, Pressure, msw},
    setup::NUM_TISSUES,
};

use stdc_stm32_rs::{
    components::dive_log::{GF_HIGH, GF_LOW},
    constants::barometric::DepthOrAltitude,
};
use stm32l4xx_hal::rtc::Rtc;

#[cfg(feature = "live_sim")]
use stdc_stm32_rs::components::LiveSimEmulationControl;
use stdc_stm32_rs::{
    algorithms::{
        helpers::datetime_to_epoch_seconds,
        rate_algorithm::{FlashLogRateAlgorithm, RateAlgorithm},
    },
    benchmarking,
    components::{
        dive_log::{
            CurrentDiveModeWithInfo, DecoAlgorithmType, LevelState, LogDiveControlDataBlock,
            LogPointData, LogPointMetadata,
        },
        spi_utils::DetailsError,
        // uart_log removed from dive mode; use rprintln! instead
    },
};

use super::{display_set_depth, millis_tim5, millis_tim5_since};
use crate::{
    LatestCalculationsState, LatestMeasurements, MEASUREMENT_BUFFER_SIZE, SensorMs5849,
    tasks::dive::{
        DiveTaskState, refresh_display_if_due, update_deco_schedule_if_due, update_dive_time_if_due,
    },
};

const DIVE_END_TOLERANCE_MILLIS: u32 = 10_000;

#[cfg(feature = "lin_exp")]
pub const DECO_ALGORITHM_TYPE: DecoAlgorithmType = DecoAlgorithmType::LinearExponential;
#[cfg(not(feature = "lin_exp"))]
pub const DECO_ALGORITHM_TYPE: DecoAlgorithmType = DecoAlgorithmType::Exponential;

pub struct DiveRuntime<const NR_GASES: usize> {
    dive_start_millis: u32,
    surface_pressure: Pa,
    deco_settings: DecoSettings<Pa>,
    gases: [GasMix<f32>; NR_GASES],
    gases_enabled: [bool; NR_GASES],
    current_gas_mode_idx: CurrentDiveModeWithInfo,
    last_measurement_millis: u32,
    last_logged_millis: u32,
    last_logged_pressure: Pa,
    max_depth: Pa,
    surfaced_since_millis: Option<u32>,
    flash_log_algorithm: FlashLogRateAlgorithm,
    task_state: DiveTaskState,
    #[cfg(feature = "online_benchmarking")]
    pub is_benchmark_profile: bool,
}

pub struct DiveFlashLog {
    pub measurement_millis: u32,
    pub pressure: Pa,
    pub measurement: DiveMeasurement<Pa>,
}

pub fn setup_dive_mode<const NR_GASES: usize>(
    flash: &mut crate::FlashDevice,
    rtc: &Rtc,
    surface_pressure: Pa,
    gases: &[GasMix<f32>; NR_GASES],
    gases_enabled: &[bool; NR_GASES],
) -> DiveRuntime<NR_GASES>
where
    [(); NR_GASES * 3]: Sized,
    [(); 24 + NR_GASES * 3]: Sized,
    [(); 4 + 24 + NR_GASES * 3]: Sized,
    [(); 4 + (24 + NR_GASES * 3)]: Sized,
    [(); 4 + (24 + NR_GASES * 3) + 0]: Sized,
    [(); 24 + NR_GASES * 3 + 0]: Sized,
{
    let dive_start_millis = millis_tim5();

    let deco_settings = DecoSettings {
        gas_density_settings: GasDensitySettings::Limit {
            limit_g_l: MAX_GAS_DENSITY,
        },
        max_deco_po2: Bar::new(1.6).to_pa(),
        gf_high: GF_HIGH.to_f32().unwrap() / 100.0,
        gf_low: GF_LOW.to_f32().unwrap() / 100.0,
        ignore_icd: false,
        last_deco_stop: msw(6.0),
    };

    let current_gas_mode_idx = CurrentDiveModeWithInfo::OC { gas_idx: 0 };

    #[cfg(all(
        feature = "live_sim_50m",
        feature = "online_benchmarking",
        target_os = "none"
    ))]
    benchmarking::log_session_start("dive.online_benchmarking.profile.mid_50m");
    #[cfg(all(
        feature = "live_sim_20m",
        feature = "online_benchmarking",
        target_os = "none"
    ))]
    benchmarking::log_session_start("dive.online_benchmarking.profile.shallow_20m");
    #[cfg(all(
        feature = "live_sim_90m",
        feature = "online_benchmarking",
        target_os = "none"
    ))]
    benchmarking::log_session_start("dive.online_benchmarking.profile.deep");
    #[cfg(feature = "online_benchmarking")]
    benchmarking::log_schema("summary.csv");

    let (start_date, start_time) = rtc.get_date_time();
    let start_epoch_seconds = datetime_to_epoch_seconds(start_date, start_time);
    let surface_interval_seconds = 0;
    let dive_number = 0;
    let surface_pressure_hpa = surface_pressure.to_hpa().to_f32() as u16;
    let surface_temperature_2 = 0;
    let ascent_rate_agg_seconds = 4;
    let dive_control_data_block = LogDiveControlDataBlock::new(
        start_epoch_seconds,
        surface_interval_seconds,
        dive_number,
        surface_pressure_hpa,
        surface_temperature_2,
        ascent_rate_agg_seconds,
        gases,
        DECO_ALGORITHM_TYPE,
    );

    let dive_control_write = crate::modes::flash_write_and_measure("flash.control.write", || {
        dive_control_data_block.write(flash)
    });
    if let Err(e) = dive_control_write {
        rprintln!("Failed writing dive control data block.");
        rprintln!("Flash error details: {:?}", e.details());
    } else if let Ok(start_addr) = dive_control_write {
        let next_pos = start_addr + (24 + NR_GASES * 3) as u32;
        if let Err(e) = crate::modes::flash_write_and_measure("flash.control.persist_pos", || {
            crate::persist_flash_log_position(flash, next_pos)
        }) {
            rprintln!(
                "Failed persisting flash position after control block: {:?}",
                e
            );
        }
    }

    DiveRuntime::<NR_GASES> {
        dive_start_millis,
        surface_pressure,
        deco_settings,
        gases: *gases,
        gases_enabled: *gases_enabled,
        current_gas_mode_idx,
        last_measurement_millis: dive_start_millis,
        last_logged_millis: dive_start_millis,
        last_logged_pressure: surface_pressure,
        max_depth: surface_pressure,
        surfaced_since_millis: None,
        flash_log_algorithm: FlashLogRateAlgorithm::default(),
        task_state: DiveTaskState::new(dive_start_millis, surface_pressure.to_msw()),
        #[cfg(feature = "online_benchmarking")]
        is_benchmark_profile: true,
    }
}

pub async fn run_dive_mode_tick<
    D: stdc_stm32_rs::components::display::LedDisplay,
    const NUM_GASES: usize,
>(
    runtime: &mut DiveRuntime<NUM_GASES>,
    display: &mut D,
    ms5849_i2c: &mut SensorMs5849,
    latest_measurements: LatestMeasurements,
    latest_calculations_state: &mut LatestCalculationsState<{ NUM_TISSUES }, Pa>,
) -> (Option<Pa>, LatestMeasurements, Option<DiveFlashLog>)
where
    [(); NUM_GASES * 3]: Sized,
    [(); 24 + NUM_GASES * 3]: Sized,
    [(); 4 + 24 + NUM_GASES * 3]: Sized,
    [(); 4 + (24 + NUM_GASES * 3)]: Sized,
    [(); 4 + (24 + NUM_GASES * 3) + 0]: Sized,
    [(); 24 + NUM_GASES * 3 + 0]: Sized,
{
    let current_millis = millis_tim5_since(runtime.dive_start_millis);
    let _ = update_dive_time_if_due(&mut runtime.task_state, runtime.dive_start_millis);

    let measurement_millis = millis_tim5();

    ms5849_i2c.read_i2c().await;
    let temperature_c = ms5849_i2c.temperature();

    let mut latest_measurements = latest_measurements;

    let flash_log: Option<DiveFlashLog>;
    let pressure = ms5849_i2c.pressure();
    match ms5849_i2c.depth_relative_or_altitude(pressure, runtime.surface_pressure) {
        Ok(DepthOrAltitude::Depth { pressure, depth }) => {
            runtime.surfaced_since_millis = None;
            latest_measurements = latest_measurements.record_environment(
                pressure,
                Some(depth),
                temperature_c,
                measurement_millis,
            );
            #[cfg(not(feature = "live_sim"))]
            rprintln!("Handling depth measurement {:?} at {:?}", pressure, depth);
            #[cfg(feature = "live_sim")]
            rprintln!(
                "PROFILE,time_ms={},pa={},msw={}",
                measurement_millis,
                pressure.to_pa().to_f32(),
                depth.to_msw().to_f32()
            );
            handle_depth_measurement(
                pressure,
                depth,
                latest_measurements.pressure.map(|sample| sample.value),
                latest_calculations_state,
                &mut runtime.last_measurement_millis,
                measurement_millis,
                &mut runtime.max_depth,
                &runtime.gases,
                &runtime.gases_enabled,
                &runtime.current_gas_mode_idx,
            );
            let deco_overlay = update_deco_schedule_if_due(
                &mut runtime.task_state,
                depth,
                measurement_millis,
                &latest_calculations_state.deco.tissue_loadings,
                &runtime.gases,
                &runtime.gases_enabled,
                &runtime.deco_settings,
                &mut latest_calculations_state.deco.deco_rate_algorithm,
            );

            #[cfg(not(feature = "live_sim"))]
            let _ = deco_overlay;

            flash_log = flash_log_tick_plan(
                &mut runtime.flash_log_algorithm,
                &mut runtime.last_logged_millis,
                &mut runtime.last_logged_pressure,
                measurement_millis,
                pressure,
                DiveMeasurement {
                    time_ms: measurement_millis as usize,
                    depth: pressure,
                    gas: runtime.current_gas_mode_idx.to_log_byte() as usize,
                },
                &latest_measurements,
            );
        }
        Ok(DepthOrAltitude::Altitude { pressure, altitude }) => {
            latest_measurements = latest_measurements.record_environment(
                pressure,
                None,
                temperature_c,
                measurement_millis,
            );
            rprintln!(
                "Dive mode pressure: {:?}, altitude: {:?}",
                pressure,
                altitude
            );
            let start = runtime.surfaced_since_millis.get_or_insert(current_millis);
            flash_log = None;
            if current_millis.wrapping_sub(*start) > DIVE_END_TOLERANCE_MILLIS {
                return (Some(pressure), latest_measurements, flash_log);
            }
        }
        Err((pressure, err)) => {
            flash_log = None;
            rprintln!(
                "Got error while reading depth for pressure {:?}: {:?}.",
                pressure,
                err
            );
        }
    }

    let _ = refresh_display_if_due(&mut runtime.task_state, display);

    (None, latest_measurements, flash_log)
}

pub fn write_flash_log<const NUM_GASES: usize>(
    runtime: &mut DiveRuntime<NUM_GASES>,
    flash: &mut crate::FlashDevice,
    flash_log: DiveFlashLog,
    latest_measurements: &LatestMeasurements,
) {
    let temperature = latest_measurements.temperature_for_log();
    let battery = latest_measurements.battery_for_log();

    let log_point_data = LogPointData::new(
        LogPointMetadata::new(true, false, LevelState::Error, [false; 4]),
        &flash_log.measurement,
        0,
        temperature,
        battery,
    );

    let write_res =
        crate::modes::flash_write_and_measure("flash.point.write", || log_point_data.write(flash));
    match write_res {
        Ok((basic_start_addr, deco_start_addr)) => {
            runtime.last_logged_millis = flash_log.measurement_millis;
            runtime.last_logged_pressure = flash_log.pressure;
            #[cfg(feature = "live_sim")]
            rprintln!(
                "FLASHLOG,time_ms={},pa={},gas={}",
                flash_log.measurement_millis,
                flash_log.pressure.to_pa().to_f32(),
                flash_log.measurement.gas,
            );

            let next_pos = deco_start_addr
                .map(|addr| addr + 8)
                .unwrap_or(basic_start_addr + 8);
            if let Err(e) = crate::modes::flash_write_and_measure("flash.point.persist_pos", || {
                crate::persist_flash_log_position(flash, next_pos)
            }) {
                rprintln!("Failed persisting flash position after log point: {:?}", e);
            }
        }
        Err(e) => {
            rprintln!("Failed writing log point data to flash: {:?}", e);
        }
    }
}

fn handle_depth_measurement<const NR_GASES: usize>(
    pressure: Pa,
    depth: msw,
    previous_pressure: Option<Pa>,
    latest_calculations_state: &mut LatestCalculationsState<{ NUM_TISSUES }, Pa>,
    last_measurement_millis: &mut u32,
    measurement_millis: u32,
    max_depth: &mut Pa,
    gases: &[GasMix<f32>; NR_GASES],
    _gases_enabled: &[bool; NR_GASES],
    current_gas_mode_idx: &CurrentDiveModeWithInfo,
) {
    benchmarking::measure_and_log("dive.rate_and_logging", || {
        display_set_depth(depth);

        let time_delta = measurement_millis.wrapping_sub(*last_measurement_millis);
        *last_measurement_millis = measurement_millis;

        if *max_depth < pressure {
            *max_depth = pressure;
        }

        let loading_pressure = previous_pressure
            .map(|prev| (prev + pressure) / 2.0)
            .unwrap_or(pressure);
        let current_gas = current_gas_mode_idx.to_fixed_gas(gases, loading_pressure);
        let delta_time = Duration::from_millis(time_delta as u64);
        update_model_state(
            &mut latest_calculations_state.deco.tissue_loadings,
            &TISSUES,
            &MVALUES,
            &current_gas,
            loading_pressure,
            &delta_time,
        );
    });

    let current_measurement = DiveMeasurement {
        time_ms: measurement_millis as usize,
        depth: pressure,
        gas: current_gas_mode_idx.to_log_byte() as usize,
    };
    latest_calculations_state.o2_tox.measurements_since_o2_calc =
        [current_measurement; MEASUREMENT_BUFFER_SIZE];
    latest_calculations_state
        .o2_tox
        .nr_measurements_since_o2_calc = 1;

    let next_o2_tox_update_millis = latest_calculations_state.o2_tox.next_o2_tox_update_millis;
    let o2_tox_update_interval_millis = benchmarking::measure_and_log("dive.o2_tox.rate", || {
        latest_calculations_state
            .o2_tox
            .o2_tox_rate_algorithm
            .next_iter(depth, depth)
            .unwrap_or(0)
    });

    if next_o2_tox_update_millis == 0 {
        latest_calculations_state.o2_tox.next_o2_tox_update_millis =
            measurement_millis.saturating_add(o2_tox_update_interval_millis);
        #[cfg(feature = "online_benchmarking")]
        benchmarking::log_decision(
            "dive.o2_tox.rate",
            false,
            Some(o2_tox_update_interval_millis),
        );
        return;
    }

    if measurement_millis < next_o2_tox_update_millis {
        #[cfg(feature = "online_benchmarking")]
        benchmarking::log_decision(
            "dive.o2_tox.rate",
            false,
            Some(next_o2_tox_update_millis - measurement_millis),
        );
        return;
    }

    latest_calculations_state.o2_tox.next_o2_tox_update_millis =
        measurement_millis.saturating_add(o2_tox_update_interval_millis);

    benchmarking::measure_and_log("dive.o2_tox", || {
        latest_calculations_state.o2_tox.o2_tox_single = calculate_toxicity_diff(
            &[current_measurement],
            gases,
            1,
            &latest_calculations_state.o2_tox.o2_tox_single,
            &O2ExposureType::Single,
            O2ToxCalculation::RevisedDHM2025,
        );
    });
    #[cfg(feature = "online_benchmarking")]
    benchmarking::log_decision("dive.o2_tox.rate", true, None);
}

fn flash_log_tick_plan(
    flash_log_algorithm: &mut FlashLogRateAlgorithm,
    last_logged_millis: &mut u32,
    last_logged_pressure: &mut Pa,
    measurement_millis: u32,
    pressure: Pa,
    measurement: DiveMeasurement<Pa>,
    latest_measurements: &LatestMeasurements,
) -> Option<DiveFlashLog> {
    let elapsed_since_last_log = measurement_millis.saturating_sub(*last_logged_millis);
    let next_log_time_delta = benchmarking::measure_and_log("flash.log.rate", || {
        flash_log_algorithm.next_iter((*last_logged_pressure, elapsed_since_last_log), pressure)
    });
    let next_iter_due_in = next_log_time_delta.map(|n| *last_logged_millis + n);
    match next_iter_due_in {
        Ok(next_iter) => {
            if measurement_millis > next_iter {
                #[cfg(feature = "online_benchmarking")]
                benchmarking::log_decision("flash.log.rate", true, None);
                let _ = latest_measurements;
                Some(DiveFlashLog {
                    measurement_millis,
                    pressure,
                    measurement,
                })
            } else {
                #[cfg(feature = "online_benchmarking")]
                benchmarking::log_decision(
                    "flash.log.rate",
                    false,
                    Some(next_iter.saturating_sub(measurement_millis)),
                );
                None
            }
        }
        Err(e) => {
            rprintln!("Failed getting next log time: {:?}", e);
            None
        }
    }
}
