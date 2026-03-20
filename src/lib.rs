#![no_std]
#![feature(const_trait_impl)]
#![feature(const_ops)]
#![feature(const_default)]
#![feature(generic_const_exprs)]

pub mod barometric;
pub mod display;
pub mod ms5849;
pub mod spi;
pub mod flash;
pub mod dive_log;

pub use ms5849::MS5849;
