use core::{cell::RefCell, fmt::Debug};

use cortex_m::interrupt::{Mutex, free};
use rtt_target::rprintln;
use stm32l4xx_hal::hal::blocking::i2c::{Read, Write, WriteRead};
use thalmann::{dive::DiveMeasurement, pressure_unit::Pa};

use stdc_stm32_rs::{
    algorithms::rate_algorithm::{FixedRateAlgorithm, RateAlgorithm},
    components::{
        MS5849,
        dive_log::{LevelState, LogPointData, LogPointMetadata},
        flash::Flash,
        uart_log::ExternalLogger,
    },
    constants::barometric::{DepthOrAltitude, SURFACE_PA},
};

use super::{SurfaceModeExit, millis_tim2, millis_tim2_since};

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
            last_logged_millis: 0,
            last_polled_millis: Self::get_last_polled_millis_for_now(),
        }
    }

    pub fn reset_for_entry(&mut self) {
        self.prev_surface_altitude = None;
        self.last_polled_millis = Self::get_last_polled_millis_for_now();
    }

    /**
     * Ensure task due on next tick
     */
    fn get_last_polled_millis_for_now() -> u32 {
        millis_tim2().wrapping_sub(SURFACE_POLL_INTERVAL_MILLIS)
    }
}

pub fn run_surface_mode_tick<I: Write + Read + WriteRead, F: Flash, L: ExternalLogger>(
    state: &mut SurfaceModeState,
    ms5849_i2c: &mut MS5849<'_, I, ()>,
    flash: &mut F,
    latest_measurements: &mut crate::LatestMeasurements,
    logger: &Mutex<RefCell<L>>,
) -> Option<SurfaceModeExit>
where
    <I as Read>::Error: Debug,
    <I as Write>::Error: Debug,
    <I as WriteRead>::Error: Debug,
{
    if millis_tim2_since(state.last_polled_millis) < SURFACE_POLL_INTERVAL_MILLIS {
        return None;
    }
    state.last_polled_millis = millis_tim2();
    ms5849_i2c.read_i2c();
    let pressure = match ms5849_i2c.current_pressure_pa() {
        Some(p) => p,
        None => {
            rprintln!("Got error while measuring pressure");
            return None;
        }
    };
    let current_measurement_millis = millis_tim2();
    let temperature_c = ms5849_i2c.temperature();
    latest_measurements.record_environment(pressure, None, temperature_c, current_measurement_millis);

    let next_flash_iter = state
        .flash_log_algorithm
        .next_iter(state.last_logged_millis, current_measurement_millis);
    let next_flash_log = next_flash_iter.map(|n| state.last_logged_millis + n);
    if let Ok(n) = next_flash_log
        && n >= current_measurement_millis
    {
        log_data_flash(
            current_measurement_millis,
            pressure,
            latest_measurements,
            flash,
            state,
        )
    }

    handle_surface_mode_tick_end(ms5849_i2c, state, pressure, logger)
}

fn handle_surface_mode_tick_end<I: Write + Read + WriteRead, L: ExternalLogger>(
    ms5849_i2c: &mut MS5849<'_, I, ()>,
    state: &mut SurfaceModeState,
    pressure: Pa,
    logger: &Mutex<RefCell<L>>,
) -> Option<SurfaceModeExit>
where
    <I as Read>::Error: Debug,
    <I as Write>::Error: Debug,
    <I as WriteRead>::Error: Debug,
{
    match ms5849_i2c.depth_relative_or_altitude(SURFACE_PA) {
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
            log_bytes(logger, b"Surface mode tick complete");
            None
        }
        Err(err) => {
            rprintln!(
                "Got error while reading depth for pressure {:?}: {:?}.",
                pressure,
                err,
            );
            None
        }
    }
}

fn log_data_flash<F: Flash>(
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
    match data.write(flash) {
        Ok(_) => {
            state.last_logged_millis = current_measurement_millis;
        }
        Err(e) => {
            rprintln!("Got error writing new measurement to flash {:?}", e);
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
