set shell := ["zsh", "-eu", "-o", "pipefail", "-c"]

features := env_var_or_default('JUST_EMBED_FEATURES', '')
host_target := env_var_or_default('JUST_HOST_TARGET', 'x86_64-apple-darwin')

default:
	just --list

c:
	cargo check --target {{host_target}} --lib --tests
	cargo check --target thumbv7em-none-eabihf --bin stdc_stm32_rs --lib

b:
	cargo build --target thumbv7em-none-eabihf --bin stdc_stm32_rs --lib

b-bench:
	cargo build --features online_benchmarking --target thumbv7em-none-eabihf --bin stdc_stm32_rs --lib

t:
	cargo test --target {{host_target}} --lib --tests

test-bench:
	cargo test --target {{host_target}} --lib --tests benchmark_tests -- --nocapture

bench:
	just sim-all

sim-small:
	cargo test --target {{host_target}} --lib benchmark_tests::benchmark_small_profile -- --nocapture

sim-large:
	cargo test --target {{host_target}} --lib benchmark_tests::benchmark_large_profile -- --nocapture

sim-deep:
	cargo test --target {{host_target}} --lib benchmark_tests::benchmark_deep_profile -- --nocapture

sim-all:
	just sim-small
	just sim-large
	just sim-deep

embed:
	# Example: just features=online_benchmarking,bluetooth,display embed
	if [[ -n "{{features}}" ]]; then cargo embed --features "{{features}}"; else cargo embed; fi

embed-bench:
	# Real sensor path + online benchmarking markers/logging
	just features=online_benchmarking embed

embed-live-sim-log-90m:
	# Live sim on the 90 m / 10 min profile.
	NO_COLOR=1 CARGO_TERM_COLOR=never JUST_EMBED_FEATURES=live_sim,live_sim_90m bash scripts/embed_live_sim_log.sh

embed-live-sim-log-90m-fixed-rate:
	# Live sim on the 90 m / 10 min profile with fixed-rate algorithms.
	NO_COLOR=1 CARGO_TERM_COLOR=never JUST_EMBED_FEATURES=live_sim,live_sim_90m,fixed_rate_algorithms bash scripts/embed_live_sim_log.sh

embed-live-sim-log-90m-exp:
	# Live sim on the 90 m / 10 min profile with pure exponential algorithms.
	NO_COLOR=1 CARGO_TERM_COLOR=never JUST_EMBED_PROFILE=exp JUST_EMBED_NO_DEFAULT_FEATURES=1 JUST_EMBED_FEATURES=display,live_sim,live_sim_90m bash scripts/embed_live_sim_log.sh

embed-live-sim-log-20m:
	# Live sim on the shorter 20 m / 2 min profile.
	NO_COLOR=1 CARGO_TERM_COLOR=never JUST_EMBED_FEATURES=live_sim,live_sim_20m bash scripts/embed_live_sim_log.sh

embed-live-sim-log-20m-fixed-rate:
	# Live sim on the shorter 20 m / 2 min profile with fixed-rate algorithms.
	NO_COLOR=1 CARGO_TERM_COLOR=never JUST_EMBED_FEATURES=live_sim,live_sim_20m,fixed_rate_algorithms bash scripts/embed_live_sim_log.sh

embed-live-sim-log-20m-exp:
	# Live sim on the shorter 20 m / 2 min profile with pure exponential algorithms.
	NO_COLOR=1 CARGO_TERM_COLOR=never JUST_EMBED_PROFILE=exp JUST_EMBED_NO_DEFAULT_FEATURES=1 JUST_EMBED_FEATURES=display,live_sim,live_sim_20m bash scripts/embed_live_sim_log.sh

embed-live-sim-log-50m:
	# Live sim on a 50m profile with multi-stop deco and staged gases (18/45 + 50/00)
	NO_COLOR=1 CARGO_TERM_COLOR=never JUST_EMBED_FEATURES=live_sim,live_sim_50m bash scripts/embed_live_sim_log.sh

embed-live-sim-log-50m-fixed-rate:
	# Live sim on a 50m profile with multi-stop deco and staged gases (18/45 + 50/00) using fixed-rate algorithms.
	NO_COLOR=1 CARGO_TERM_COLOR=never JUST_EMBED_FEATURES=live_sim,live_sim_50m,fixed_rate_algorithms bash scripts/embed_live_sim_log.sh

embed-live-sim-log-50m-exp:
	# Live sim on a 50m profile with multi-stop deco and staged gases (18/45 + 50/00) using pure exponential algorithms.
	NO_COLOR=1 CARGO_TERM_COLOR=never JUST_EMBED_PROFILE=exp JUST_EMBED_NO_DEFAULT_FEATURES=1 JUST_EMBED_FEATURES=display,live_sim,live_sim_50m bash scripts/embed_live_sim_log.sh

ci:
	just c
	just t
	just bench
