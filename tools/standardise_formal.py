#!/usr/bin/env python3
"""Turn ``formal.jsonl`` into a strict, standard-compliant table.

``formal.jsonl`` is the author's own dump: it contains the bare tokens ``NaN``,
``Infinity`` and ``-Infinity``, which are **not** valid JSON (RFC 8259 has no
such literals).  Most Python tooling accepts them, but ``jq``, JavaScript,
Rust, Go and many CSV/parquet loaders do not, and a silent ``NaN -> 0``
substitution would corrupt the statistics.

This script writes ``formal_standardised.csv`` next to the original with:

* every non-finite value written as an **empty cell**, never as ``0``;
* an explicit marker column ``_nonfinite`` that names the affected fields and
  the exact kind of non-finite value, e.g.
  ``recovery=NaN;role_switches=NaN;first_contact_time_rr=Infinity``;
* a distinct, empty representation for genuine JSON ``null`` (missing), so
  "missing" and "non-finite" never collapse into each other.

``--verify`` reloads the CSV and proves the round-trip: every cell is compared
against the original record field by field.

    python tools/standardise_formal.py            # write the CSV
    python tools/standardise_formal.py --verify   # write, then prove round-trip
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "02_Perception_Robustness" / "results" / "formal.jsonl"
DST = ROOT / "02_Perception_Robustness" / "results" / "formal_standardised.csv"
MARKER = "_nonfinite"

NONFINITE_TEXT = {math.inf: "Infinity", -math.inf: "-Infinity"}


def kind(value):
    """'NaN' / 'Infinity' / '-Infinity' for a non-finite float, else None."""
    if isinstance(value, float) and not math.isfinite(value):
        return "NaN" if math.isnan(value) else NONFINITE_TEXT[value]
    return None


def cell(value):
    """Text for one cell (never renders a non-finite value as a number)."""
    if value is None:
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return repr(value)
    if isinstance(value, int):
        return str(value)
    return str(value)


def load_rows(path):
    rows = []
    with path.open(encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def build(rows, out):
    fields = []
    seen = set()
    for row in rows:
        for key in row:
            if key not in seen:
                seen.add(key)
                fields.append(key)
    fields.append(MARKER)
    with out.open("w", encoding="utf-8-sig", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            record = {}
            marks = []
            for key in fields:
                if key == MARKER:
                    continue
                value = row.get(key)
                k = kind(value)
                if k is not None:
                    marks.append(f"{key}={k}")
                record[key] = cell(value)
            record[MARKER] = ";".join(marks)
            writer.writerow(record)
    return fields


def verify(rows, fields):
    """Reload the CSV and compare every cell with the original record."""
    bad = 0
    with DST.open(encoding="utf-8-sig", newline="") as fh:
        reader = csv.DictReader(fh)
        if reader.fieldnames != fields:
            print("  [FAIL] header changed on reload")
            return 1
        for index, (row, got) in enumerate(zip(rows, reader)):
            marks = [m for m in got[MARKER].split(";") if m]
            marked = {}
            for mark in marks:
                key, _, text = mark.partition("=")
                marked[key] = text
            for key in fields:
                if key == MARKER:
                    continue
                value = row.get(key)
                text = got[key]
                if key in marked:
                    # the marker must describe a genuinely non-finite value
                    if kind(value) != marked[key]:
                        print(f"  [FAIL] row {index} field {key}: marker "
                              f"{marked[key]} but value {value!r}")
                        bad += 1
                    if text != "":
                        print(f"  [FAIL] row {index} field {key}: non-finite "
                              f"value leaked as {text!r}")
                        bad += 1
                    continue
                if text != cell(value):
                    print(f"  [FAIL] row {index} field {key}: {text!r} != "
                          f"{cell(value)!r}")
                    bad += 1
                if bad > 20:
                    print("  (stopping after 20 mismatches)")
                    return bad
    return bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--verify", action="store_true",
                    help="reload the CSV and prove the round-trip")
    args = ap.parse_args()

    rows = load_rows(SRC)
    fields = build(rows, DST)
    size_mb = DST.stat().st_size / 1048576
    print(f"wrote {DST.relative_to(ROOT)}")
    print(f"  rows    : {len(rows)}")
    print(f"  columns : {len(fields)} (including the marker '{MARKER}')")
    print(f"  size    : {size_mb:.2f} MiB")

    flagged = sum(1 for row in rows if any(kind(v) for v in row.values()))
    print(f"  rows carrying at least one non-finite value: {flagged}")

    if not args.verify:
        return 0

    bad = verify(rows, fields)
    if bad:
        print(f"ROUND-TRIP: FAIL ({bad} mismatches)")
        return 1
    print("ROUND-TRIP: PASS - every cell re-read matches the original record; "
          "no non-finite value was turned into a number")
    return 0


if __name__ == "__main__":
    sys.exit(main())
