#!/usr/bin/env python3

from __future__ import annotations

import csv
import re
import statistics
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path


SAMPLE_RE = re.compile(r"bench_sample,label=([^,\s]+),cycles=(\d+),nanos=(\d+)")
DECISION_RE = re.compile(r"bench_decision,label=([^,\s]+),due=(true|false)(?:,skipped_ms=(\d+))?")
SESSION_RE = re.compile(r"bench_session,name=([^,\s]+)")
SCHEMA_RE = re.compile(r"bench_schema,name=([^,\s]+)")

VALID_SAMPLE_LABELS = {
    "dive.rate_and_logging",
    "dive.deco_schedule",
    "dive.deco_schedule.rate",
    "dive.o2_tox",
    "dive.o2_tox.rate",
    "dive.display_refresh",
    "flash.control.write",
    "flash.init.reset_pos",
    "flash.point.write",
    "flash.serial.write",
    "task.mode.tick.loop",
}

VALID_DECISION_LABELS = {
    "dive.deco_schedule.rate",
    "dive.o2_tox.rate",
    "dive.display_refresh.rate",
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
    skipped_ms = [decision.skipped_ms for decision in due]
    skipped_seconds = [value / 1000.0 for value in skipped_ms]

    return {
        "count": len(decisions),
        "due_count": len(due),
        "not_due_count": len(decisions) - len(due),
        "sum_skipped_ms": sum(skipped_ms),
        "min_skipped_ms": min(skipped_ms) if skipped_ms else 0,
        "median_skipped_ms": median_float([float(value) for value in skipped_ms]),
        "avg_skipped_ms": (sum(skipped_ms) / len(skipped_ms)) if skipped_ms else 0.0,
        "max_skipped_ms": max(skipped_ms) if skipped_ms else 0,
        "sum_skipped_seconds": sum(skipped_seconds),
        "min_skipped_seconds": min(skipped_seconds) if skipped_seconds else 0.0,
        "median_skipped_seconds": median_float(skipped_seconds),
        "avg_skipped_seconds": (sum(skipped_seconds) / len(skipped_seconds)) if skipped_seconds else 0.0,
        "max_skipped_seconds": max(skipped_seconds) if skipped_seconds else 0.0,
    }


def extract_records(log_text: str) -> tuple[list[Sample], list[Decision], list[str], list[str]]:
    samples: list[Sample] = []
    decisions: list[Decision] = []
    sessions: list[str] = []
    schemas: list[str] = []

    for match in SAMPLE_RE.finditer(log_text):
        label = match.group(1)
        if label in VALID_SAMPLE_LABELS:
            samples.append(Sample(label, int(match.group(2)), int(match.group(3))))
    for match in DECISION_RE.finditer(log_text):
        label = match.group(1)
        if label in VALID_DECISION_LABELS:
            decisions.append(
                Decision(
                    label,
                    match.group(2) == "true",
                    int(match.group(3) or 0),
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
    samples, decisions, sessions, schemas = extract_records(log_text)

    output_dir = log_path.with_suffix("")
    output_dir = output_dir.parent / f"{output_dir.name}.tables"
    output_dir.mkdir(parents=True, exist_ok=True)

    write_summary_csv(output_dir / "summary.csv", samples)
    write_decision_csv(output_dir / "decision.csv", decisions)
    write_manifest(output_dir / "manifest.txt", log_path, samples, decisions, sessions, schemas)

    print(f"summary table: {output_dir / 'summary.csv'}")
    print(f"decision table: {output_dir / 'decision.csv'}")
    print(f"manifest: {output_dir / 'manifest.txt'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())