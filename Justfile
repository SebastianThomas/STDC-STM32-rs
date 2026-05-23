set shell := ["zsh", "-eu", "-o", "pipefail", "-c"]

features := env_var_or_default('JUST_EMBED_FEATURES', '')

default:
	just --list

c:
	cargo check --target x86_64-apple-darwin --lib --tests
	cargo check --target thumbv7em-none-eabihf --bin stdc_stm32_rs --lib

b:
	cargo build --target thumbv7em-none-eabihf --bin stdc_stm32_rs --lib

b-bench:
	cargo build --features online_benchmarking --target thumbv7em-none-eabihf --bin stdc_stm32_rs --lib

t:
	cargo test --target x86_64-apple-darwin --lib --tests

test-bench:
	cargo test --target x86_64-apple-darwin --lib --tests benchmark_tests -- --nocapture

bench:
	just sim-all

sim-small:
	cargo test --target x86_64-apple-darwin --lib benchmark_tests::benchmark_small_profile -- --nocapture

sim-large:
	cargo test --target x86_64-apple-darwin --lib benchmark_tests::benchmark_large_profile -- --nocapture

sim-deep:
	cargo test --target x86_64-apple-darwin --lib benchmark_tests::benchmark_deep_profile -- --nocapture

sim-all:
	just sim-small
	just sim-large
	just sim-deep

embed:
	# Example: just features=online_benchmarking,bluetooth,display embed
	if [[ -n "{{features}}" ]]; then cargo embed --features "{{features}}"; else cargo embed; fi

embed-bench:
	just features=online_benchmarking embed

ci:
	just c
	just t
	just bench
