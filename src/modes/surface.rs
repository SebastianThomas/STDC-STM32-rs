use rtt_target::rprintln;
use stdc_diving_algorithms::{dive::DiveMeasurement, pressure_unit::Pa};

use stdc_stm32_rs::{
    algorithms::rate_algorithm::{FixedRateAlgorithm, RateAlgorithm},
    components::{
        dive_log::{LevelState, LogPointData, LogPointMetadata},
        flash::Flash,
    },
    constants::barometric::DepthOrAltitude,
};

use crate::DEFAULT_SURFACE_PRESSURE;

use super::{SurfaceModeExit, millis_tim5, millis_tim5_since};

const SURFACE_POLL_INTERVAL_MILLIS: u32 = 5 * 1000;
const SURFACE_LOG_INTERVAL_MILLIS: u32 = 2 * 60 * 1000;

pub struct SurfaceModeState {
    prev_surface_altitude: Option<Pa>,
    flash_log_algorithm: FixedRateAlgorithm,
    last_logged_millis: u32,
    last_polled_millis: u32,
}

impl SurfaceModeState {
    pub fn new() -> Self {
        Self {
            prev_surface_altitude: None,
            flash_log_algorithm: FixedRateAlgorithm::new(SURFACE_LOG_INTERVAL_MILLIS),
            last_logged_millis: millis_tim5(),
            last_polled_millis: Self::get_last_polled_millis_for_now(),
        }
    }

    pub fn reset_for_entry(&mut self) {
        self.prev_surface_altitude = None;
        self.last_logged_millis = millis_tim5();
        self.last_polled_millis = Self::get_last_polled_millis_for_now();
    }

    /**
     * Ensure task due on next tick
     */
    fn get_last_polled_millis_for_now() -> u32 {
        millis_tim5().wrapping_sub(SURFACE_POLL_INTERVAL_MILLIS)
    }
}

pub async fn run_surface_mode_tick(
    state: &mut SurfaceModeState,
    ms5849_i2c: &mut crate::SensorMs5849,
    latest_measurements: crate::LatestMeasurements,
) -> (
    Option<SurfaceModeExit>,
    crate::LatestMeasurements,
    Option<(u32, Pa)>,
) {
    let millis_since = millis_tim5_since(state.last_polled_millis);
    if millis_since < SURFACE_POLL_INTERVAL_MILLIS {
        return (None, latest_measurements, None);
    }
    rprintln!(
        "Updating measurements (not up to date: {} ms)",
        millis_since
    );
    let current_measurement_millis = millis_tim5();
    ms5849_i2c.read_i2c().await;
    let pressure = match ms5849_i2c.current_pressure_pa() {
        Some(p) => p,
        None => {
            rprintln!("Got error while measuring pressure");
            return (None, latest_measurements, None);
        }
    };
    let temperature_c = ms5849_i2c.temperature();
    let latest_measurements = latest_measurements.record_environment(
        pressure,
        None,
        temperature_c,
        current_measurement_millis,
    );

    let next_flash_iter = state
        .flash_log_algorithm
        .next_iter(state.last_logged_millis, current_measurement_millis);
    let next_flash_log = next_flash_iter.map(|n| state.last_logged_millis + n);
    state.last_polled_millis = current_measurement_millis;

    let flash_log = if let Ok(n) = next_flash_log {
        if current_measurement_millis >= n {
            Some((current_measurement_millis, pressure))
        } else {
            None
        }
    } else {
        None
    };

    (
        handle_surface_mode_tick_end(ms5849_i2c, state),
        latest_measurements,
        flash_log,
    )
}

fn handle_surface_mode_tick_end(
    ms5849_i2c: &mut crate::SensorMs5849,
    state: &mut SurfaceModeState,
) -> Option<SurfaceModeExit> {
    let pressure = ms5849_i2c.pressure();
    match ms5849_i2c.depth_relative_or_altitude(
        pressure,
        state
            .prev_surface_altitude
            .unwrap_or(DEFAULT_SURFACE_PRESSURE),
    ) {
        Ok(DepthOrAltitude::Depth { pressure, depth }) => {
            rprintln!(
                "Starting Dive: Pressure: {:?}, depth: {:?}",
                pressure,
                depth
            );
            let adjusted_surface_pressure =
                (pressure + state.prev_surface_altitude.unwrap_or(pressure)) / 2.0;
            Some(SurfaceModeExit::Dive(adjusted_surface_pressure))
        }
        Ok(DepthOrAltitude::Altitude { pressure, altitude }) => {
            state.prev_surface_altitude = Some(pressure);
            rprintln!(
                "Surface mode pressure: {:?}, altitude: {:?}",
                pressure,
                altitude
            );
            None
        }
        Err(err) => {
            rprintln!("Got error while reading depth: {:?}.", err,);
            None
        }
    }
}

pub fn log_data_flash<F: Flash>(
    current_measurement_millis: u32,
    pressure: Pa,
    latest_measurements: &crate::LatestMeasurements,
    flash: &mut F,
    state: &mut SurfaceModeState,
) {
    let temperature = latest_measurements.temperature_for_log();
    let battery = latest_measurements.battery_for_log();
    let data = LogPointData::new(
        LogPointMetadata::new(false, false, LevelState::Error, [false; 4]),
        &DiveMeasurement {
            time_ms: current_measurement_millis as usize,
            depth: pressure,
            gas: 0b01111111,
        },
        0,
        temperature,
        battery,
    );
    let write_res =
        crate::modes::flash_write_and_measure("flash.point.write", || data.write(flash));
    match write_res {
        Ok(_) => {
            state.last_logged_millis = current_measurement_millis;
        }
        Err(e) => {
            rprintln!("Got error writing new measurement to flash {:?}", e);
        }
    }
}
