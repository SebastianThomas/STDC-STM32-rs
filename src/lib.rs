#![no_std]
#![feature(const_trait_impl)]
#![feature(const_ops)]
#![feature(const_default)]

pub mod ms5849;
pub mod spi;
pub mod display;

pub use ms5849::ms5849_pressure::MS5849;
