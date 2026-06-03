#!/usr/bin/env python3
"""
For each flash-logged dive point, interpolates the captured dive profile at that
timestamp and writes the signed depth difference (flash_depth - profile_depth).

Inputs  (from a .tables/ directory produced by extract_bench_tables.py):
  profile.csv            — high-rate depth samples from the RTT PROFILE lines
  flash_log_profile.csv  — one row per flash-written log point

Outputs:
  flash_profile_diff.csv         — one row per flash point
  flash_profile_diff_summary.csv — aggregate statistics over all flash points

  depth_diff_m > 0  →  flash recorded the diver as deeper than the profile shows
  depth_diff_m < 0  →  flash recorded shallower
"""

from __future__ import annotations

import csv
import statistics
import sys
from pathlib import Path


def _read_profile(path: Path) -> list[tuple[float, float]]:
    records: list[tuple[float, float]] = []
    with path.open(newline="") as f:
        for row in csv.DictReader(line for line in f if not line.startswith("#")):
            try:
                records.append((float(row["time_ms"]), float(row["depth_m"])))
            except (KeyError, ValueError):
                continue
    records.sort(key=lambda x: x[0])
    return records


def _interpolate(profile: list[tuple[float, float]], t: float) -> float:
    """Linear interpolation; clamps to the first/last sample if t is out of range."""
    if t <= profile[0][0]:
        return profile[0][1]
    if t >= profile[-1][0]:
        return profile[-1][1]
    lo, hi = 0, len(profile) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if profile[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    t0, d0 = profile[lo]
    t1, d1 = profile[hi]
    return d0 + (d1 - d0) * (t - t0) / (t1 - t0)


def generate(tables_dir: Path) -> tuple[Path, Path]:
    """Compute diffs and write both output CSVs. Returns (diff_path, summary_path)."""
    profile = _read_profile(tables_dir / "profile.csv")

    flash_rows: list[tuple[int, float, int]] = []
    with (tables_dir / "flash_log_profile.csv").open(newline="") as f:
        for row in csv.DictReader(line for line in f if not line.startswith("#")):
            try:
                flash_rows.append((
                    int(row["time_ms"]),
                    float(row["depth_m"]),
                    int(row["gas_idx"]),
                ))
            except (KeyError, ValueError):
                continue

    diffs: list[float] = []
    diff_path = tables_dir / "flash_profile_diff.csv"
    with diff_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_ms", "depth_flash_m", "depth_profile_m", "depth_diff_m", "gas_idx"])
        for time_ms, flash_depth, gas_idx in flash_rows:
            profile_depth = _interpolate(profile, float(time_ms))
            diff = flash_depth - profile_depth
            diffs.append(diff)
            writer.writerow([
                time_ms,
                f"{flash_depth:.3f}",
                f"{profile_depth:.3f}",
                f"{diff:.3f}",
                gas_idx,
            ])

    summary_path = tables_dir / "flash_profile_diff_summary.csv"
    with summary_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["count", "min_diff_m", "max_diff_m", "avg_diff_m", "median_diff_m"])
        if diffs:
            writer.writerow([
                len(diffs),
                f"{min(diffs):.3f}",
                f"{max(diffs):.3f}",
                f"{sum(diffs) / len(diffs):.3f}",
                f"{statistics.median(diffs):.3f}",
            ])
        else:
            writer.writerow([0, "", "", "", ""])

    return diff_path, summary_path


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {Path(sys.argv[0]).name} TABLES_DIR", file=sys.stderr)
        return 2

    tables_dir = Path(sys.argv[1]).expanduser().resolve()
    for name in ("profile.csv", "flash_log_profile.csv"):
        p = tables_dir / name
        if not p.exists():
            print(f"error: {p} not found", file=sys.stderr)
            return 1

    if not (tables_dir / "profile.csv").stat().st_size or \
       not (tables_dir / "flash_log_profile.csv").stat().st_size:
        print("warning: one or both input files are empty — skipping diff", file=sys.stderr)
        return 0

    diff_path, summary_path = generate(tables_dir)
    print(f"flash profile diff:         {diff_path}")
    print(f"flash profile diff summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
