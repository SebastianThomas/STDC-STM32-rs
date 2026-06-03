#!/usr/bin/env python3
"""
For each flash-logged dive point, interpolates the captured dive profile at that
timestamp and writes the signed pressure difference (flash_pa - profile_pa).

Comparison is performed in Pa (raw sensor pressure) to avoid freshwater/saltwater
density assumptions.  The diff is then expressed in metres using freshwater density
(ρ = 1000 kg/m³, g = 9.81 m/s²) for human-readable output.

Inputs  (from a .tables/ directory produced by extract_bench_tables.py):
  profile.csv            — high-rate depth+pressure samples from the RTT PROFILE lines
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

FRESHWATER_PA_PER_M = 9810.0  # ρ=1000 kg/m³, g=9.81 m/s²


def _read_profile_pa(path: Path) -> list[tuple[float, float]]:
    """Return (time_ms, pa) pairs sorted by time.

    Falls back to reconstructing Pa from depth_m for legacy CSVs that lack the pa column
    (profile.csv pre-dates the pa column: depth_m is in msw, 1 msw = 10130 Pa gauge).
    """
    records: list[tuple[float, float]] = []
    with path.open(newline="") as f:
        for row in csv.DictReader(line for line in f if not line.startswith("#")):
            try:
                t = float(row["time_ms"])
                if "pa" in row and row["pa"]:
                    pa = float(row["pa"])
                else:
                    pa = 100000.0 + float(row["depth_m"]) * 10130.0
                records.append((t, pa))
            except (KeyError, ValueError):
                continue
    records.sort(key=lambda x: x[0])
    return records


def _read_flash_pa(path: Path) -> list[tuple[int, float, int]]:
    """Return (time_ms, pa, gas_idx) tuples.

    Falls back to reconstructing Pa from depth_m for legacy CSVs
    (flash_log_profile.csv pre-dates the pa column: depth_m = (pa-101325)/10057.7).
    """
    records: list[tuple[int, float, int]] = []
    with path.open(newline="") as f:
        for row in csv.DictReader(line for line in f if not line.startswith("#")):
            try:
                t = int(row["time_ms"])
                if "pa" in row and row["pa"]:
                    pa = float(row["pa"])
                else:
                    pa = 101325.0 + float(row["depth_m"]) * 10057.7
                records.append((t, pa, int(row["gas_idx"])))
            except (KeyError, ValueError):
                continue
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
    t0, v0 = profile[lo]
    t1, v1 = profile[hi]
    return v0 + (v1 - v0) * (t - t0) / (t1 - t0)


def generate(tables_dir: Path) -> tuple[Path, Path]:
    """Compute Pa-based diffs and write both output CSVs. Returns (diff_path, summary_path)."""
    profile = _read_profile_pa(tables_dir / "profile.csv")
    flash_rows = _read_flash_pa(tables_dir / "flash_log_profile.csv")

    diffs_m: list[float] = []
    diff_path = tables_dir / "flash_profile_diff.csv"
    with diff_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "time_ms", "pa_flash", "pa_profile", "pa_diff",
            "depth_flash_m", "depth_profile_m", "depth_diff_m", "gas_idx",
        ])
        for time_ms, flash_pa, gas_idx in flash_rows:
            profile_pa = _interpolate(profile, float(time_ms))
            diff_pa = flash_pa - profile_pa
            diff_m = diff_pa / FRESHWATER_PA_PER_M
            diffs_m.append(diff_m)
            writer.writerow([
                time_ms,
                f"{flash_pa:.1f}",
                f"{profile_pa:.1f}",
                f"{diff_pa:.1f}",
                f"{max(0.0, (flash_pa - 101325.0) / FRESHWATER_PA_PER_M):.3f}",
                f"{max(0.0, (profile_pa - 101325.0) / FRESHWATER_PA_PER_M):.3f}",
                f"{diff_m:.3f}",
                gas_idx,
            ])

    summary_path = tables_dir / "flash_profile_diff_summary.csv"
    with summary_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["count", "min_diff_m", "max_diff_m", "avg_diff_m", "median_diff_m"])
        if diffs_m:
            writer.writerow([
                len(diffs_m),
                f"{min(diffs_m):.3f}",
                f"{max(diffs_m):.3f}",
                f"{sum(diffs_m) / len(diffs_m):.3f}",
                f"{statistics.median(diffs_m):.3f}",
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
