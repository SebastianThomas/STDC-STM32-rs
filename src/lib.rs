#![cfg_attr(not(all(test, not(target_os = "none"))), no_std)]
#![feature(const_trait_impl)]
#![feature(const_default)]
#![allow(incomplete_features)]
#![allow(unused_features)]
#![feature(generic_const_exprs)]

pub mod algorithms;
pub mod constants;
pub mod benchmarking;
pub mod macros;

#[cfg(target_os = "none")]
pub mod components;
#[cfg(target_os = "none")]
pub mod protocols;
#[cfg(target_os = "none")]
pub mod stm32;

#[cfg(all(test, not(target_os = "none")))]
#[path = "components/battery_status/battery_status_values.rs"]
pub mod battery_status_values;
#[cfg(all(test, not(target_os = "none")))]
#[path = "components/dive_log.rs"]
pub mod dive_log_host;

#[cfg(all(test, not(target_os = "none")))]
extern crate std;
#[cfg(all(test, not(target_os = "none")))]
mod tests;
