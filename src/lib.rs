#![no_std]
#![feature(const_trait_impl)]
#![feature(const_ops)]
#![feature(const_default)]

pub mod barometric;
pub mod display;
pub mod ms5849;
pub mod spi;

pub use ms5849::MS5849;
