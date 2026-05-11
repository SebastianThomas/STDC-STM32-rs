use crate::BATTERY_UPDATE_INTERVAL_MILLIS;
use crate::modes::millis_tim2;
use rtic_monotonics::fugit::ExtU32;
use rtic_monotonics::stm32::prelude::*;

pub fn calc_battery_status(
    battery_status: &mut crate::BatteryGauge<'static>,
    latest_measurements: &mut crate::LatestMeasurements,
) {
    if let Ok(b) = battery_status.read_snapshot() {
        let taken_millis = millis_tim2();
        latest_measurements.record_battery(b, taken_millis);
    }
}

pub async fn wait_for_next_battery_update() {
    crate::Mono::delay(BATTERY_UPDATE_INTERVAL_MILLIS.millis().into()).await;
}
