#![cfg_attr(not(all(test, not(target_os = "none"))), no_std)]
#![feature(const_trait_impl)]
#![feature(const_default)]
#![allow(incomplete_features)]
#![allow(unused_features)]
#![feature(generic_const_exprs)]

#[cfg(all(
	feature = "live_sim",
	not(any(feature = "live_sim_90m", feature = "live_sim_20m", feature = "live_sim_50m"))
))]
compile_error!("enable exactly one live-sim profile feature: live_sim_90m, live_sim_20m, or live_sim_50m");

#[cfg(all(feature = "live_sim", feature = "live_sim_90m", feature = "live_sim_20m"))]
compile_error!("enable only one live-sim profile feature at a time");

#[cfg(all(feature = "live_sim", feature = "live_sim_90m", feature = "live_sim_50m"))]
compile_error!("enable only one live-sim profile feature at a time");

#[cfg(all(feature = "live_sim", feature = "live_sim_20m", feature = "live_sim_50m"))]
compile_error!("enable only one live-sim profile feature at a time");

pub mod algorithms;
pub mod benchmarking;
pub mod constants;
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
