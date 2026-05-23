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

embed-no-progress:
	# Same as embed, but with progress bars disabled for cleaner log capture
	if [[ -n "{{features}}" ]]; then cargo embed --disable-progressbars --features "{{features}}"; else cargo embed --disable-progressbars; fi

embed-clean-log:
	# Capture raw output, strip terminal control codes, and drop progress-only lines
	mkdir -p target
	raw_log=target/rtt-clean-$(date +%Y%m%d-%H%M%S).raw.log; \
	clean_log=target/rtt-clean-$(date +%Y%m%d-%H%M%S).log; \
	NO_COLOR=1 CARGO_TERM_COLOR=never just features=online_benchmarking,live_sim embed-no-progress 2>&1 | tee "$raw_log" || true; \
	perl -pe 's/\r\n?/\n/g; s/\e\[[0-9;?]*[ -\/]*[@-~]//g; s/\e\][^\a]*(?:\a|\e\\)//g; s/\e[@-_]//g' "$raw_log" \
	| sed '/^[[:space:]]*$/d;/Erasing/d;/Programming/d;/Config default/d;/Target [[:space:]]/d;/^░/d;/^⠁/d;/^⠉/d;/^⠋/d;/^⠙/d;/^⠚/d;/^⠒/d;/^⠂/d;/^⠤/d;/^⠦/d;/^⠖/d;/^⠓/d;/^⠊/d;/^⠈/d;/^⠐/d;/^⠠/d;/^⠸/d;/^⠴/d;/^⠧/d;/^⠇/d;/^⠏/d' \
	| awk 'NF { print; blank = 0; next } !blank { print; blank = 1 }' > "$clean_log"; \
	python3 scripts/extract_bench_tables.py "$raw_log"; \
	echo "raw log: $raw_log"; \
	echo "clean log: $clean_log"; \
	echo "tables dir: ${raw_log%.raw.log}.tables"

embed-bench:
	# Real sensor path + online benchmarking markers/logging
	just features=online_benchmarking embed

embed-live-sim:
	# Live emulated dive path on device + online benchmarking markers/logging
	just features=online_benchmarking,live_sim embed

embed-bench-log:
	mkdir -p target
	raw_log=target/rtt-bench-$(date +%Y%m%d-%H%M%S).log; \
	clean_log=${raw_log%.log}.clean.log; \
	NO_COLOR=1 CARGO_TERM_COLOR=never just features=online_benchmarking embed-no-progress 2>&1 | tee "$raw_log" || true; \
	perl -pe 's/\r\n?/\n/g; s/\e\[[0-9;?]*[ -\/]*[@-~]//g; s/\e\][^\a]*(?:\a|\e\\)//g; s/\e[@-_]//g' "$raw_log" > "$clean_log"; \
	python3 scripts/extract_bench_tables.py "$raw_log"; \
	echo "raw log: $raw_log"; \
	echo "clean log: $clean_log"; \
	echo "tables dir: ${raw_log%.log}.tables"

embed-live-sim-log:
	mkdir -p target
	raw_log=target/rtt-live-sim-$(date +%Y%m%d-%H%M%S).log; \
	clean_log=${raw_log%.log}.clean.log; \
	NO_COLOR=1 CARGO_TERM_COLOR=never just features=online_benchmarking,live_sim embed-no-progress 2>&1 | tee "$raw_log" || true; \
	perl -pe 's/\r\n?/\n/g; s/\e\[[0-9;?]*[ -\/]*[@-~]//g; s/\e\][^\a]*(?:\a|\e\\)//g; s/\e[@-_]//g' "$raw_log" > "$clean_log"; \
	python3 scripts/extract_bench_tables.py "$raw_log"; \
	echo "raw log: $raw_log"; \
	echo "clean log: $clean_log"; \
	echo "tables dir: ${raw_log%.log}.tables"
	
ci:
	just c
	just t
	just bench
