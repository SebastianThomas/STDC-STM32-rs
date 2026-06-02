#!/usr/bin/env python3

from __future__ import annotations

import bisect
import csv
import re
import statistics
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path


SAMPLE_RE = re.compile(r"bench_sample,label=([^,\s]+),cycles=(\d+),nanos=(\d+)")
DECISION_RE = re.compile(r"(?m)^bench_decision,label=([^,\s]+),due=(true|false)(?:,skipped_ms=(\d+))?")
SESSION_RE = re.compile(r"bench_session,name=([^,\s]+)")
SCHEMA_RE = re.compile(r"bench_schema,name=([^,\s]+)")
# Various debug prints include time and depth in slightly different formats. Cover common cases:
#  - EmulatedDivePoint debug: "time_ms: 12345" and "depth_msw: msw(12.34)"
#  - DiveMeasurement debug: "time_ms: 12345" and "depth: Pa(101325.0)" (convert Pa->msw if possible)
#  - Generic prints like "... at 12.34 m" or "depth=12.34"
TIME_MS_RE = re.compile(r"time_ms\s*[:=]\s*(\d+)")
DEPTH_MEASUREMENT_RE = re.compile(
    r"Handling depth measurement\s+Pa\(([-+]?[0-9]*\.?[0-9]+)\)\s+at\s+msw\(([-+]?[0-9]*\.?[0-9]+)\)"
)
DEPTH_MSW_RE = re.compile(r"depth_msw\s*[:=]\s*msw\(([-+]?[0-9]*\.?[0-9]+)\)")
DEPTH_PA_RE = re.compile(r"depth\s*[:=]\s*Pa\(([-+]?[0-9]*\.?[0-9]+)\)")
DEPTH_M_RE = re.compile(r"(?:at|depth=)\s*([-+]?[0-9]*\.?[0-9]+)\s*m")
ANSI_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]|\x1b\][^\a]*(?:\a|\x1b\\)|\x1b[@-_]")
DIVE_PRESSURE_RE = re.compile(r"Dive mode pressure:\s*Pa\(([-+]?[0-9]*\.?[0-9]+)\)")
PROFILE_RE = re.compile(r"PROFILE\s+time_ms=(\d+)\s+pa=([-+]?[0-9]*\.?[0-9]+)\s+msw=([-+]?[0-9]*\.?[0-9]+)")
PROFILE_OR_HANDLING_RE = re.compile(
    r"Handling depth measurement\s+Pa\(([-+]?[0-9]*\.?[0-9]+)\)\s+at\s+msw\(([-+]?[0-9]*\.?[0-9]+)\)"
    r"|PROFILE\s+time_ms=(\d+)\s+pa=([-+]?[0-9]*\.?[0-9]+)\s+msw=([-+]?[0-9]*\.?[0-9]+)"
)

VALID_SAMPLE_LABELS = {
    "dive.rate_and_logging",
    "dive.deco_schedule",
    "dive.deco_schedule.rate",
    "dive.o2_tox",
    "dive.o2_tox.rate",
    "dive.display_refresh",
    "flash.control.write",
    "flash.init.reset_pos",
    "flash.log.rate",
    "flash.point.write",
    "flash.serial.write",
    "task.mode.tick.loop",
}

VALID_DECISION_LABELS = {
    "dive.deco_schedule.rate",
    "dive.o2_tox.rate",
    "dive.display_refresh.rate",
    "flash.log.rate",
}


@dataclass(frozen=True)
class Sample:
    label: str
    cycles: int
    nanos: int


@dataclass(frozen=True)
class Decision:
    label: str
    due: bool
    skipped_ms: int
    time_ms: int = 0  # wall-clock anchor from nearest preceding time_ms token (0 = unknown)


def median_int(values: list[int]) -> int:
    if not values:
        return 0
    return int(statistics.median(values))


def median_float(values: list[float]) -> float:
    if not values:
        return 0.0
    return float(statistics.median(values))


def summarize_samples(samples: list[Sample]) -> dict[str, int]:
    cycles = [sample.cycles for sample in samples]
    nanos = [sample.nanos for sample in samples]
    return {
        "count": len(samples),
        "sum_cycles": sum(cycles),
        "min_cycles": min(cycles) if cycles else 0,
        "median_cycles": median_int(cycles),
        "avg_cycles": sum(cycles) // len(cycles) if cycles else 0,
        "max_cycles": max(cycles) if cycles else 0,
        "sum_nanos": sum(nanos),
        "min_nanos": min(nanos) if nanos else 0,
        "median_nanos": median_int(nanos),
        "avg_nanos": sum(nanos) // len(nanos) if nanos else 0,
        "max_nanos": max(nanos) if nanos else 0,
    }


def summarize_decisions(decisions: list[Decision]) -> dict[str, float | int]:
    due = [decision for decision in decisions if decision.due]
    not_due = [decision for decision in decisions if not decision.due]

    # --- wall-clock seconds: derived from intervals between consecutive due=true events ---
    # Sort due events by their time_ms anchor, ignoring records with no anchor (time_ms==0).
    due_with_time = sorted(
        [d for d in due if d.time_ms > 0],
        key=lambda d: d.time_ms,
    )
    due_times_ms = [d.time_ms for d in due_with_time]
    # Differences between successive due events give real inter-run wall-clock intervals.
    intervals_ms = [
        due_times_ms[i] - due_times_ms[i - 1]
        for i in range(1, len(due_times_ms))
        if due_times_ms[i] - due_times_ms[i - 1] > 0  # skip zero/negative (clock artefacts)
    ]
    intervals_s = [v / 1000.0 for v in intervals_ms]

    sum_s = sum(intervals_s) if intervals_s else 0.0

    return {
        "count": len(decisions),
        "due_count": len(due),
        "not_due_count": len(decisions) - len(due),
        # wall-clock ms from inter-due intervals (accurate, no double-counting)
        "sum_skipped_ms": sum(intervals_ms) if intervals_ms else 0,
        "min_skipped_ms": min(intervals_ms) if intervals_ms else 0,
        "median_skipped_ms": median_float([float(v) for v in intervals_ms]),
        "avg_skipped_ms": (sum(intervals_ms) / len(intervals_ms)) if intervals_ms else 0.0,
        "max_skipped_ms": max(intervals_ms) if intervals_ms else 0,
        # wall-clock seconds from inter-due intervals (accurate, no double-counting)
        "sum_skipped_seconds": sum_s,
        "min_skipped_seconds": min(intervals_s) if intervals_s else 0.0,
        "median_skipped_seconds": median_float(intervals_s),
        "avg_skipped_seconds": (sum_s / len(intervals_s)) if intervals_s else 0.0,
        "max_skipped_seconds": max(intervals_s) if intervals_s else 0.0,
    }


def extract_records(log_text: str) -> tuple[list[Sample], list[Decision], list[str], list[str]]:
    samples: list[Sample] = []
    decisions: list[Decision] = []
    sessions: list[str] = []
    schemas: list[str] = []
    # Ensure concatenated bench tokens are split onto their own lines so partial writes
    # don't produce torn numeric fields (e.g. "skipped_ms=12345bench_sample,...").
    log_text = re.sub(r"(bench_sample,label=|bench_decision,label=|bench_session,name=|bench_schema,name=)", r"\n\1", log_text)

    for match in SAMPLE_RE.finditer(log_text):
        label = match.group(1)
        if label in VALID_SAMPLE_LABELS:
            samples.append(Sample(label, int(match.group(2)), int(match.group(3))))
    # Build a global map of (log_position, time_ms) for all time_ms tokens in the
    # preprocessed text.  RTT corruption can land PROFILE lines *after* the
    # bench_decision they belong to, so a backward-only window misses them.
    # Using nearest-by-position (in either direction) anchors almost every decision.
    _time_map: list[tuple[int, int]] = [
        (m.start(), int(m.group(1))) for m in TIME_MS_RE.finditer(log_text)
    ]
    _tm_positions: list[int] = [t[0] for t in _time_map]

    def _nearest_time_anchor(pos: int) -> int:
        if not _time_map:
            return 0
        i = bisect.bisect_left(_tm_positions, pos)
        candidates: list[tuple[int, int]] = []
        if i > 0:
            candidates.append(_time_map[i - 1])
        if i < len(_time_map):
            candidates.append(_time_map[i])
        return min(candidates, key=lambda t: abs(t[0] - pos))[1]

    for match in DECISION_RE.finditer(log_text):
        label = match.group(1)
        if label in VALID_DECISION_LABELS:
            time_anchor = _nearest_time_anchor(match.start())
            decisions.append(
                Decision(
                    label,
                    match.group(2) == "true",
                    int(match.group(3) or 0),
                    time_anchor,
                )
            )
    for match in SESSION_RE.finditer(log_text):
        sessions.append(match.group(1))
    for match in SCHEMA_RE.finditer(log_text):
        schemas.append(match.group(1))

    return samples, decisions, sessions, schemas


def write_summary_csv(path: Path, samples: list[Sample]) -> None:
    by_label: dict[str, list[Sample]] = defaultdict(list)
    for sample in samples:
        by_label[sample.label].append(sample)

    with path.open("w", newline="") as file:
        file.write("# summary.csv aggregates benchmark timing samples extracted from the log.\n")
        file.write(
            "# label names the benchmarked operation. All rows are treated as single measurements because logs do not contain offline window batches.\n"
        )
        file.write(
            "section,label,count,sum_cycles,min_cycles,median_cycles,avg_cycles,max_cycles,sum_ns,min_ns,median_ns,avg_ns,max_ns\n"
        )
        writer = csv.writer(file)
        for label in sorted(by_label):
            stats = summarize_samples(by_label[label])
            writer.writerow([
                "single",
                label,
                stats["count"],
                stats["sum_cycles"],
                stats["min_cycles"],
                stats["median_cycles"],
                stats["avg_cycles"],
                stats["max_cycles"],
                stats["sum_nanos"],
                stats["min_nanos"],
                stats["median_nanos"],
                stats["avg_nanos"],
                stats["max_nanos"],
            ])


def write_decision_csv(path: Path, decisions: list[Decision]) -> None:
    by_label: dict[str, list[Decision]] = defaultdict(list)
    for decision in decisions:
        by_label[decision.label].append(decision)

    with path.open("w", newline="") as file:
        file.write("# decision.csv summarizes scheduler decisions extracted from the log.\n")
        file.write(
            "# skipped values come from the RTT bench_decision lines, so the table tracks milliseconds rather than replay iterations.\n"
        )
        file.write(
            "label,count,due_count,not_due_count,sum_skipped_ms,min_skipped_ms,median_skipped_ms,avg_skipped_ms,max_skipped_ms,sum_skipped_seconds,min_skipped_seconds,median_skipped_seconds,avg_skipped_seconds,max_skipped_seconds\n"
        )
        writer = csv.writer(file)
        for label in sorted(by_label):
            stats = summarize_decisions(by_label[label])
            writer.writerow([
                label,
                stats["count"],
                stats["due_count"],
                stats["not_due_count"],
                stats["sum_skipped_ms"],
                stats["min_skipped_ms"],
                f"{stats['median_skipped_ms']:.1f}",
                f"{stats['avg_skipped_ms']:.1f}",
                stats["max_skipped_ms"],
                f"{stats['sum_skipped_seconds']:.3f}",
                f"{stats['min_skipped_seconds']:.3f}",
                f"{stats['median_skipped_seconds']:.3f}",
                f"{stats['avg_skipped_seconds']:.3f}",
                f"{stats['max_skipped_seconds']:.3f}",
            ])


def write_profile_csv(path: Path, log_text: str) -> None:
    # Try to find surface pressure to convert Pa -> meters. Default to 101325 Pa.
    surface_pa = 101325.0
    m = re.search(r"Start Pressure:\s*([0-9]+\.?[0-9]*)\s*Pa", log_text)
    if m:
        try:
            surface_pa = float(m.group(1))
        except Exception:
            pass

    records: list[tuple[int, float]] = []
    used_times: set[int] = set()

    # The live RTT stream can interleave partial writes, which occasionally tears a PROFILE line
    # and drops digits from the depth value. Scan the stream in order and prefer the preceding
    # handling-depth value when it is available.
    pending_handling_depth: float | None = None

    for match in PROFILE_OR_HANDLING_RE.finditer(log_text):
        if match.group(1) is not None:
            try:
                pending_handling_depth = float(match.group(2))
            except Exception:
                pending_handling_depth = None
            continue

        try:
            time_ms = int(match.group(3))
            profile_depth = float(match.group(5))
        except Exception:
            pending_handling_depth = None
            continue

        depth_m = pending_handling_depth if pending_handling_depth is not None else profile_depth
        pending_handling_depth = None

        if not (0.0 <= depth_m <= 200.0):
            continue
        if time_ms not in used_times:
            records.append((time_ms, depth_m))
            used_times.add(time_ms)

    if records:
        records.sort(key=lambda x: x[0])
        with path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time_ms", "depth_m"])
            for t, d in records:
                writer.writerow([t, f"{d:.3f}"])
        return

    for match in DEPTH_MEASUREMENT_RE.finditer(log_text):
        try:
            msw_val = float(match.group(2))
        except Exception:
            continue
        start = max(0, match.start() - 200)
        end = min(len(log_text), match.end() + 200)
        time_search = TIME_MS_RE.search(log_text, start, end)
        if not time_search:
            # no time nearby — skip, we require time for reliable extraction
            continue
        time_ms = int(time_search.group(1))
        # sanity-check msw
        if not (0.0 <= msw_val <= 200.0):
            continue
        if time_ms not in used_times:
            records.append((time_ms, msw_val))
            used_times.add(time_ms)

    # 2) Find time_ms anchors and look ahead for msw/Pa within a window.
    for tmatch in TIME_MS_RE.finditer(log_text):
        try:
            time_ms = int(tmatch.group(1))
        except Exception:
            continue
        if time_ms in used_times:
            continue
        start = tmatch.end()
        end = min(len(log_text), start + 400)
        window = log_text[start:end]
        dm = DEPTH_MSW_RE.search(window)
        dp = DEPTH_PA_RE.search(window)
        if dm:
            try:
                depth_m = float(dm.group(1))
                if 0.0 <= depth_m <= 200.0:
                    records.append((time_ms, depth_m))
                used_times.add(time_ms)
                continue
            except Exception:
                pass
        if dp:
            try:
                pa = float(dp.group(1))
                depth_m = max(0.0, (pa - surface_pa) / 10057.7)
                if 0.0 <= depth_m <= 200.0:
                    records.append((time_ms, depth_m))
                used_times.add(time_ms)
                continue
            except Exception:
                pass

    # 3) Catch 'Dive mode pressure' tokens and find nearby time_ms.
    for dmatch in DIVE_PRESSURE_RE.finditer(log_text):
        try:
            pa = float(dmatch.group(1))
        except Exception:
            continue
        start = max(0, dmatch.start() - 200)
        end = min(len(log_text), dmatch.end() + 200)
        time_search = TIME_MS_RE.search(log_text, start, end)
        if time_search:
            time_ms = int(time_search.group(1))
        else:
            # require a time anchor for dive-pressure entries
            continue
        depth_m = max(0.0, (pa - surface_pa) / 10057.7)
        if not (0.0 <= depth_m <= 200.0):
            continue
        if time_ms not in used_times:
            records.append((time_ms, depth_m))
            used_times.add(time_ms)

    # Sort records by time
    records.sort(key=lambda x: x[0])

    # Write CSV
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_ms", "depth_m"])
        for t, d in records:
            writer.writerow([t, f"{d:.3f}"])


def derive_profile_tag(sessions: list[str]) -> str | None:
    if not sessions:
        return None

    session = ANSI_RE.sub("", sessions[-1])
    if ".shallow_20m" in session:
        return "20m-2min"
    if ".mid_50m" in session:
        return "50m-15min"
    if ".deep" in session:
        return "90m-10min"
    return None



def write_manifest(path: Path, log_path: Path, samples: list[Sample], decisions: list[Decision], sessions: list[str], schemas: list[str]) -> None:
    with path.open("w") as file:
        file.write(f"log_file={log_path}\n")
        if sessions:
            file.write(f"bench_session={sessions[-1]}\n")
        if schemas:
            file.write(f"bench_schema={schemas[-1]}\n")
        file.write(f"sample_count={len(samples)}\n")
        file.write(f"decision_count={len(decisions)}\n")


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {Path(sys.argv[0]).name} LOG_FILE", file=sys.stderr)
        return 2

    log_path = Path(sys.argv[1]).expanduser().resolve()
    if not log_path.exists():
        print(f"error: log file not found: {log_path}", file=sys.stderr)
        return 1

    log_text = log_path.read_text(errors="replace")
    # Normalize carriage-returns used by progress bars into real line breaks and strip ANSI.
    cleaned_log = ANSI_RE.sub("", log_text).replace('\r', '\n')
    samples, decisions, sessions, schemas = extract_records(cleaned_log)

    output_dir = log_path.with_suffix("")
    profile_tag = derive_profile_tag(sessions)
    if profile_tag:
        output_dir = output_dir.parent / f"{output_dir.name}.{profile_tag}.tables"
    else:
        output_dir = output_dir.parent / f"{output_dir.name}.tables"
    output_dir.mkdir(parents=True, exist_ok=True)

    write_summary_csv(output_dir / "summary.csv", samples)
    write_decision_csv(output_dir / "decision.csv", decisions)
    write_profile_csv(output_dir / "profile.csv", cleaned_log)
    write_manifest(output_dir / "manifest.txt", log_path, samples, decisions, sessions, schemas)

    print(f"summary table: {output_dir / 'summary.csv'}")
    print(f"decision table: {output_dir / 'decision.csv'}")
    print(f"profile table: {output_dir / 'profile.csv'}")
    print(f"manifest: {output_dir / 'manifest.txt'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())