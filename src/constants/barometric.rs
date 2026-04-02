use thalmann::pressure_unit::{Bar, Pa, Pressure, hPa, kPa, msw};

pub const SURFACE_PA: Pa = Pa::new(101325.0);
pub const SURFACE_HPA: hPa = SURFACE_PA.to_hpa();
pub const SURFACE_KPA: kPa = SURFACE_PA.to_kpa();
pub const SURFACE_BAR: Bar = SURFACE_PA.to_bar();

pub const FEET_TO_METERS: f32 = 0.3048;
pub const ALT_PER_FOOT: f32 = 145366.45;

pub const KG_M2_FRESH_WATER: u16 = 1000;
pub const KG_M2_SEA_WATER: u16 = 1029;

/**
* In meters
*/
pub enum DepthOrAltitude {
    Depth { pressure: Pa, depth: msw },
    Altitude { pressure: Pa, altitude: f32 },
}

// (R*L)/(g*M), the exponent in the barometric formula
pub const RLGM: f32 = 0.190284;
