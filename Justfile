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

embed:
	# Example: just features=online_benchmarking,bluetooth,display embed
	if [[ -n "{{features}}" ]]; then cargo embed --features "{{features}}"; else cargo embed; fi

embed-bench:
	just features=online_benchmarking embed

ci:
	just c
	just t
	just bench
