#!/usr/bin/env bash
set -u

mkdir -p target
raw_log=target/rtt-live-sim-$(date +%Y%m%d-%H%M%S).log
clean_log=${raw_log%.log}.clean.log
interrupted=0

finish() {
	perl -pe 's{\r\n?}{\n}g; s{\e\[[0-9;?]*[ -/]*[@-~]}{}g; s{\e\][^\a]*(?:\a|\e\\)}{}g; s{\e[@-_]}{}g' "$raw_log" > "$clean_log"
	python3 scripts/extract_bench_tables.py "$raw_log"
	echo "raw log: $raw_log"
	echo "clean log: $clean_log"
	echo "tables dir: ${raw_log%.log}.tables"
}

on_signal() {
	interrupted=1
	trap '' INT TERM
}

trap on_signal INT TERM

if [[ -n "${JUST_EMBED_FEATURES:-}" ]]; then
	env JUST_EMBED_FEATURES="$JUST_EMBED_FEATURES" cargo embed --disable-progressbars --features "$JUST_EMBED_FEATURES" 2>&1 | tee "$raw_log"
else
	cargo embed --disable-progressbars 2>&1 | tee "$raw_log"
fi

cargo_status=$?

finish

if [[ $interrupted -eq 1 ]]; then
	exit 130
fi

exit "$cargo_status"
