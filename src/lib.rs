#![no_std]

 #![feature(const_trait_impl)]

pub mod ms5849;
pub mod pressure_unit;

pub use ms5849::ms5849_pressure::MS5849;

