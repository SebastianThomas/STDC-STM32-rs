#![no_std]
#![feature(const_trait_impl)]
// #![feature(const_ops)]
#![feature(const_default)]
#![feature(generic_const_exprs)]

#[cfg(all(test, not(target_os = "none")))]
extern crate std;

pub mod algorithms;
pub mod barometric;
pub mod macros;

#[cfg(target_os = "none")]
pub mod components;

#[cfg(target_os = "none")]
pub mod spi;

#[cfg(all(test, not(target_os = "none")))]
mod tests;
