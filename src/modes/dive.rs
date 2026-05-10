use core::{cell::RefCell, fmt::Debug};

use cortex_m::interrupt::{Mutex, free};
use rtt_target::rprintln;
use stdc_diving_algorithms::{
    deco_algorithm::{DecoSettings, MVALUES, TISSUES},
    dive::{DiveMeasurement, DiveProfile},
    gas::{GasDensitySettings, GasMix, MAX_GAS_DENSITY, TissuesLoading},
    loadings_from_dive_profile,
    o2tox::{O2ExposureType, O2ToxCalculation, calculate_toxicity_diff},
    pressure_unit::{Bar, Pa, Pressure, msw},
    setup::NUM_TISSUES,
};
use stm32l4xx_hal::{
    hal::blocking::i2c::{Read, Write, WriteRead},
    rtc::Rtc,
};

use stdc_stm32_rs::{
    algorithms::{
        helpers::datetime_to_epoch_seconds,
        rate_algorithm::{DynamicDiffAimdRateAlgorithm, RateAlgorithm},
    },
    components::{
        MS5849,
        dive_log::{
            CurrentDiveModeWithInfo, DecoAlgorithmType, LevelState, LogDiveControlDataBlock,
            LogPointData, LogPointMetadata,
        },
        flash::Flash,
        spi_utils::DetailsError,
        uart_log::ExternalLogger,
    },
    constants::barometric::DepthOrAltitude,
};

use crate::{
    LatestCalculationsState, LatestMeasurements, MEASUREMENT_BUFFER_SIZE,
    tasks::dive::{
        DiveTaskState, refresh_display_if_due, update_deco_schedule_if_due, update_dive_time_if_due,
    },
};

use super::{POWER_CUT_UNSAFE_FLASH_WRITE, power_cut_mark_safe, power_cut_mark_unsafe};
use super::{display_set_depth, millis_tim2, millis_tim2_since};

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
    loading: TissuesLoading<{ NUM_TISSUES }, Pa>,
    current_gas_mode_idx: CurrentDiveModeWithInfo,
    last_measurement_millis: u32,
    last_logged_millis: u32,
    last_logged_pressure: Pa,
    max_depth: Pa,
    surfaced_since_millis: Option<u32>,
    flash_log_algorithm: DynamicDiffAimdRateAlgorithm<Pa>,
    task_state: DiveTaskState,
}

pub fn setup_dive_mode<F: Flash, L: ExternalLogger, const NR_GASES: usize>(
    flash: &mut F,
    rtc: &Rtc,
    logger: &Mutex<RefCell<L>>,
    surface_pressure: Pa,
    gases: &[GasMix<f32>; NR_GASES],
) -> DiveRuntime<NR_GASES>
where
    [(); NR_GASES * 3]: Sized,
    [(); 24 + NR_GASES * 3]: Sized,
    [(); 4 + 24 + NR_GASES * 3]: Sized,
    [(); 4 + (24 + NR_GASES * 3)]: Sized,
    [(); 4 + (24 + NR_GASES * 3) + 0]: Sized,
    [(); 24 + NR_GASES * 3 + 0]: Sized,
{
    let dive_start_millis = millis_tim2();

    let deco_settings = DecoSettings {
        gas_density_settings: GasDensitySettings::Limit {
            limit: MAX_GAS_DENSITY.to_pa(),
        },
        max_deco_po2: Bar::new(1.6).to_pa(),
    };

    let dive_profile = DiveProfile {
        dive_id: 1,
        max_depth: surface_pressure,
        gases: *gases,
        measurements: [DiveMeasurement {
            depth: surface_pressure,
            time_ms: dive_start_millis as usize,
            gas: 0,
        }],
    };

    let loading = loadings_from_dive_profile(&TISSUES, &dive_profile, &MVALUES, surface_pressure);
    let current_gas_mode_idx = CurrentDiveModeWithInfo::OC { gas_idx: 0 };

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

    power_cut_mark_unsafe(POWER_CUT_UNSAFE_FLASH_WRITE);
    let dive_control_write = dive_control_data_block.write(flash);
    power_cut_mark_safe(POWER_CUT_UNSAFE_FLASH_WRITE);
    if let Err(e) = dive_control_write {
        log_bytes(logger, b"Failed writing dive control data block.");
        log_bytes(logger, e.details().as_bytes());
    }

    DiveRuntime::<NR_GASES> {
        dive_start_millis,
        surface_pressure,
        deco_settings,
        gases: *gases,
        loading,
        current_gas_mode_idx,
        last_measurement_millis: dive_start_millis,
        last_logged_millis: dive_start_millis,
        last_logged_pressure: surface_pressure,
        max_depth: surface_pressure,
        surfaced_since_millis: None,
        flash_log_algorithm: DynamicDiffAimdRateAlgorithm::new::<3, 4>(
            5000,
            20000,
            Bar::new(0.2).to_pa(),
            Bar::new(1.0).to_pa(),
            Bar::new(0.2).to_pa(),
        ),
        task_state: DiveTaskState::new(dive_start_millis),
    }
}

pub fn run_dive_mode_tick<
    D: stdc_stm32_rs::components::display::LedDisplay,
    I: Write + Read + WriteRead,
    F: Flash,
    L: ExternalLogger,
    const NUM_GASES: usize,
>(
    runtime: &mut DiveRuntime<NUM_GASES>,
    display: &mut D,
    ms5849_i2c: &mut MS5849<'_, I, ()>,
    flash: &mut F,
    latest_measurements: &mut LatestMeasurements,
    latest_calculations_state: &mut LatestCalculationsState<{ NUM_TISSUES }, Pa>,
    logger: &Mutex<RefCell<L>>,
) -> Option<Pa>
where
    <I as Read>::Error: Debug,
    <I as Write>::Error: Debug,
    <I as WriteRead>::Error: Debug,
    [(); NUM_GASES * 3]: Sized,
    [(); 24 + NUM_GASES * 3]: Sized,
    [(); 4 + 24 + NUM_GASES * 3]: Sized,
    [(); 4 + (24 + NUM_GASES * 3)]: Sized,
    [(); 4 + (24 + NUM_GASES * 3) + 0]: Sized,
    [(); 24 + NUM_GASES * 3 + 0]: Sized,
{
    let current_millis = millis_tim2_since(runtime.dive_start_millis);
    let _ = update_dive_time_if_due(&mut runtime.task_state, runtime.dive_start_millis, logger);

    let measurement_millis = millis_tim2();
    ms5849_i2c.read_i2c();
    let temperature_c = ms5849_i2c.temperature();

    match ms5849_i2c.depth_relative_or_altitude(runtime.surface_pressure) {
        Ok(DepthOrAltitude::Depth { pressure, depth }) => {
            runtime.surfaced_since_millis = None;
            latest_measurements.record_environment(
                pressure,
                Some(depth),
                temperature_c,
                measurement_millis,
            );
            handle_depth_measurement(
                &mut runtime.flash_log_algorithm,
                flash,
                pressure,
                depth,
                latest_measurements,
                latest_calculations_state,
                &mut runtime.last_measurement_millis,
                measurement_millis,
                &mut runtime.last_logged_millis,
                &mut runtime.last_logged_pressure,
                &mut runtime.max_depth,
                &mut runtime.loading,
                &runtime.gases,
                &runtime.current_gas_mode_idx,
            );
        }
        Ok(DepthOrAltitude::Altitude { pressure, altitude }) => {
            latest_measurements.record_environment(
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
            if current_millis.wrapping_sub(*start) > DIVE_END_TOLERANCE_MILLIS {
                return Some(pressure);
            }
        }
        Err((pressure, err)) => {
            rprintln!(
                "Got error while reading depth for pressure {:?}: {:?}.",
                pressure,
                err
            );
        }
    }

    let _ = update_deco_schedule_if_due(
        &mut runtime.task_state,
        &runtime.loading,
        &runtime.gases,
        &runtime.deco_settings,
        flash,
        logger,
    );

    let _ = refresh_display_if_due(&mut runtime.task_state, display, logger);

    None
}

fn handle_depth_measurement<
    R: RateAlgorithm<Pa, Pa, u32>,
    F: Flash,
    const NR_GASES: usize,
    const NUM_TISSUES: usize,
>(
    flash_log_algorithm: &mut R,
    flash: &mut F,
    pressure: Pa,
    depth: msw,
    latest_measurements: &mut crate::LatestMeasurements,
    latest_calculations_state: &mut LatestCalculationsState<NUM_TISSUES, Pa>,
    last_measurement_millis: &mut u32,
    measurement_millis: u32,
    last_logged_millis: &mut u32,
    last_logged_pressure: &mut Pa,
    max_depth: &mut Pa,
    loading: &mut TissuesLoading<NUM_TISSUES, Pa>,
    gases: &[GasMix<f32>; NR_GASES],
    current_gas_mode_idx: &CurrentDiveModeWithInfo,
) {
    rprintln!("Measuring Pressure: {:?}, depth: {:?}", pressure, depth);
    display_set_depth(depth);

    let measurement = DiveMeasurement {
        time_ms: measurement_millis as usize,
        depth: pressure,
        gas: current_gas_mode_idx.to_log_byte() as usize,
    };

    let time_delta = measurement_millis - *last_measurement_millis;
    *last_measurement_millis = measurement_millis;

    if *max_depth < pressure {
        *max_depth = pressure;
    }

    let current_gas = current_gas_mode_idx.to_fixed_gas(gases, pressure);
    loading.tick(
        time_delta.try_into().unwrap_or(u16::MAX),
        pressure,
        &current_gas,
    );
    latest_calculations_state.tissue_loadings = loading.clone();

    flash_log_tick(
        flash,
        flash_log_algorithm,
        last_logged_millis,
        last_logged_pressure,
        measurement_millis,
        pressure,
        &measurement,
        latest_measurements,
    );

    // TODO: Decompression Algorithm

    // TODO: O2Tox Calculation
    {
        let measurements_since_latest = latest_calculations_state.o2_tox.measurements_since_o2_calc;
        let nr_measurements = latest_calculations_state
            .o2_tox
            .nr_measurementss_since_o2_calc;
        latest_calculations_state.o2_tox.o2_tox_single = calculate_toxicity_diff(
            &measurements_since_latest[..nr_measurements],
            gases,
            1,
            &latest_calculations_state.o2_tox.o2_tox_single,
            &O2ExposureType::Single,
            O2ToxCalculation::RevisedDHM2025,
        );
        latest_calculations_state.o2_tox.measurements_since_o2_calc =
            [measurements_since_latest[nr_measurements - 1]; MEASUREMENT_BUFFER_SIZE];
        latest_calculations_state
            .o2_tox
            .nr_measurementss_since_o2_calc = 1;
    }
}

fn flash_log_tick<R: RateAlgorithm<Pa, Pa, u32>, F: Flash>(
    flash: &mut F,
    flash_log_algorithm: &mut R,
    last_logged_millis: &mut u32,
    last_logged_pressure: &mut Pa,
    measurement_millis: u32,
    pressure: Pa,
    measurement: &DiveMeasurement<Pa>,
    latest_measurements: &LatestMeasurements,
) {
    let temperature = latest_measurements.temperature_for_log();
    let battery = latest_measurements.battery_for_log();

    let next_log_time_delta = flash_log_algorithm.next_iter(*last_logged_pressure, pressure);
    let next_iter_due_in = next_log_time_delta.map(|n| *last_logged_millis + n);
    match next_iter_due_in {
        Ok(next_iter) => {
            if measurement_millis > next_iter {
                let deco_obligation = false;
                let level_state = LevelState::Error;
                let ascent_rate = 0;
                let log_point_data = LogPointData::new(
                    LogPointMetadata::new(true, deco_obligation, level_state, [false; 4]),
                    &measurement,
                    ascent_rate,
                    temperature,
                    battery,
                );
                power_cut_mark_unsafe(POWER_CUT_UNSAFE_FLASH_WRITE);
                let write_res = log_point_data.write(flash);
                power_cut_mark_safe(POWER_CUT_UNSAFE_FLASH_WRITE);
                match write_res {
                    Ok(_) => {
                        *last_logged_millis = measurement_millis;
                        *last_logged_pressure = pressure;
                    }
                    Err(e) => {
                        rprintln!("Failed writing log point data to flash: {:?}", e);
                    }
                }
            }
        }
        Err(e) => {
            rprintln!("Failed getting next log time: {:?}", e);
        }
    }
}

fn log_bytes<L: ExternalLogger>(logger: &Mutex<RefCell<L>>, bytes: &[u8]) {
    free(|cs| {
        if let Err(_) = logger.borrow(cs).borrow_mut().log_bytes(bytes) {
            rprintln!("Failed logging bytes to UART");
        }
    });
}
