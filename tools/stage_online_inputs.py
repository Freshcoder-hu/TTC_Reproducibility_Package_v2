#!/usr/bin/env python3
"""Stage the single authoritative ``formal.jsonl`` for the online analysis.

The 2,340-cell corpus is stored **once**, in
``02_Perception_Robustness/results/formal.jsonl``.  The online-measurement
scripts in ``03_Online_Warning_and_Timing`` read it from their own
``results/formal.jsonl`` and two of those scripts are fingerprint-guarded
(they hash each other), so the path cannot be rewritten inside them without
breaking the guard.

This tool therefore stages the corpus into that path on demand - as a hard
link when the platform allows it (no extra disk space, no extra bytes in the
archive), otherwise as a plain copy - and removes it again with ``--clean``.

    python tools/stage_online_inputs.py          # stage
    python tools/stage_online_inputs.py --clean  # remove the staged link

Nothing else in the package is modified.
"""
from __future__ import annotations

import argparse
import hashlib
import os
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
AUTHORITATIVE = ROOT / "02_Perception_Robustness" / "results" / "formal.jsonl"
TARGET = ROOT / "03_Online_Warning_and_Timing" / "results" / "formal.jsonl"


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--clean", action="store_true",
                    help="remove the staged link instead of creating it")
    args = ap.parse_args()

    if not AUTHORITATIVE.is_file():
        print(f"ERROR: authoritative corpus missing: {AUTHORITATIVE}")
        return 1

    if args.clean:
        if TARGET.exists():
            TARGET.unlink()
            print(f"removed {TARGET.relative_to(ROOT)}")
        else:
            print("nothing to clean")
        return 0

    if TARGET.exists():
        same = sha256(TARGET) == sha256(AUTHORITATIVE)
        print(f"already staged at {TARGET.relative_to(ROOT)} "
              f"({'identical' if same else 'DIFFERENT - remove it first'})")
        return 0 if same else 1

    TARGET.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(AUTHORITATIVE, TARGET)
        how = "hard link (shares the same bytes on disk)"
    except OSError:
        shutil.copy2(AUTHORITATIVE, TARGET)
        how = "copy (hard link not supported on this volume)"

    digest = sha256(TARGET)
    print(f"staged {how}")
    print(f"  from : {AUTHORITATIVE.relative_to(ROOT)}")
    print(f"  to   : {TARGET.relative_to(ROOT)}")
    print(f"  sha256 {digest}")
    print(f"  bytes  {TARGET.stat().st_size}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
