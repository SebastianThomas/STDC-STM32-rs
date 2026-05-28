#!/usr/bin/env python3
import csv
import sys
from pathlib import Path


def format_sec(ms_val):
    try:
        v = float(ms_val)
    except Exception:
        return ms_val
    return f"{v/1000:.3f}"


def fix_file(path: Path):
    text = path.read_text()
    lines = text.splitlines()
    # separate leading comments
    comments = []
    rest = []
    for ln in lines:
        if ln.startswith('#') or ln.strip() == '':
            comments.append(ln)
        else:
            rest.append(ln)

    if not rest:
        print('No CSV data found in', path)
        return

    reader = csv.reader(rest)
    rows = list(reader)
    header = rows[0]

    # expected column names
    ms_names = ['sum_skipped_ms', 'min_skipped_ms', 'median_skipped_ms', 'avg_skipped_ms', 'max_skipped_ms']
    sec_names = ['sum_skipped_seconds', 'min_skipped_seconds', 'median_skipped_seconds', 'avg_skipped_seconds', 'max_skipped_seconds']

    # map indices
    try:
        ms_idxs = [header.index(n) for n in ms_names]
        sec_idxs = [header.index(n) for n in sec_names]
    except ValueError as e:
        print('Header missing expected columns:', e)
        return

    # recompute seconds columns
    out_rows = [header]
    for r in rows[1:]:
        # ensure row is long enough
        if len(r) < len(header):
            r = r + [''] * (len(header) - len(r))
        for mi, si in zip(ms_idxs, sec_idxs):
            r[si] = format_sec(r[mi])
        out_rows.append(r)

    # write back preserving comments
    out_lines = []
    out_lines.extend(comments)
    # use csv to format
    from io import StringIO
    buf = StringIO()
    writer = csv.writer(buf, lineterminator='\n')
    for r in out_rows:
        writer.writerow(r)
    out_lines.extend(buf.getvalue().splitlines())

    path.write_text('\n'.join(out_lines) + '\n')
    print('Rewrote', path)


if __name__ == '__main__':
    p = Path(sys.argv[1]) if len(sys.argv) > 1 else Path('target/rtt-live-sim-20260527-205021.tables/decision.csv')
    if not p.exists():
        print('File not found:', p)
        sys.exit(2)
    fix_file(p)
