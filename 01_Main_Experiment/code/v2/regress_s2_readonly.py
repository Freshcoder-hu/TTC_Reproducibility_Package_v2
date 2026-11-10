"""Run the Stage-2 acceptance suite **without writing into results_v2_stage2**.

The Stage-3 instruction is that pre-existing ``results_*`` must not be modified.
``v2.acceptance_s2`` writes its transcript and CSVs into ``results_v2_stage2``;
this wrapper repoints the *write* locations to a scratch directory while leaving
every *read* location (the whole-trajectory reference in
``results_v2_stage2/reference``) on the archive.  It is a read-only regression
check of the Stage-2 suite against the Stage-3 code.

    python -X utf8 -m v2.regress_s2_readonly
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from v2 import acceptance_s2 as A  # noqa: E402

ARCHIVE = ROOT / "results_v2_stage2"
SCRATCH = ROOT / "results_v2_stage3" / "_regress_s2"


def main():
    SCRATCH.mkdir(parents=True, exist_ok=True)
    (SCRATCH / "behaviour").mkdir(parents=True, exist_ok=True)
    # IMPORTANT: load the SHIPPED parameter record from the archive *before*
    # repointing OUT, otherwise load_shipped() finds no calibration_summary.json
    # and silently falls back to the stale defaults baked into acceptance_s2.
    A.load_shipped()
    # writes -> scratch ; reads of the reference stay on the archive
    A.OUT = SCRATCH
    A.REF = ARCHIVE / "reference"
    A.BEH = SCRATCH / "behaviour"
    sys.argv = ["acceptance_s2"] + sys.argv[1:]
    rc = A.main()
    print(f"\n[read-only regression] transcript -> {SCRATCH}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
