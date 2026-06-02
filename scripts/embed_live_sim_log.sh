#!/usr/bin/env bash
# Builds, flashes, and runs a live-sim profile, capturing RTT output.
#
# cargo embed runs the firmware and streams the RTT "Terminal" channel through
# its TUI.  Piping to tee captures everything; the finish() step strips VT100
# cursor-control codes so extract_bench_tables.py can parse the result.
set -u

mkdir -p target
result_name=rtt-live-sim
if [[ "${JUST_EMBED_FEATURES:-}" == *fixed_rate_algorithms* ]]; then
	result_name=rtt-live-sim-fixed
fi
if [[ "${JUST_EMBED_PROFILE:-}" == "exp" ]]; then
	result_name=${result_name}-exp
fi
raw_log=target/${result_name}-$(date +%Y%m%d-%H%M%S).log
clean_log=${raw_log%.log}.clean.log
interrupted=0

finish() {
	perl -0777 -pe \
		's{\r\n?}{\n}g;
		 s{\e\[[0-9;?]*[ -/]*[@-~]}{}g;
		 s{\e\][^\a]*(?:\a|\e\\)}{}g;
		 s{\e[@-_]}{}g' \
		"$raw_log" > "$clean_log"
	python3 scripts/extract_bench_tables.py "$raw_log"
	echo "raw log:   $raw_log"
	echo "clean log: $clean_log"
}

on_signal() {
	interrupted=1
	trap '' INT TERM
}

trap on_signal INT TERM

embed_args=()
if [[ -n "${JUST_EMBED_FEATURES:-}" ]]; then
	embed_args+=(--features "${JUST_EMBED_FEATURES}")
fi
if [[ -n "${JUST_EMBED_NO_DEFAULT_FEATURES:-}" ]]; then
	embed_args+=(--no-default-features)
fi

cargo embed "${embed_args[@]}" 2>&1 | tee "$raw_log"

run_status=${PIPESTATUS[0]}

finish

if [[ $interrupted -eq 1 ]]; then
	exit 130
fi

exit "$run_status"
