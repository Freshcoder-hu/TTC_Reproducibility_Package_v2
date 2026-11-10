#!/usr/bin/env python3
"""Second-kind verification: compare *numbers*, not bytes.

The shipped package has two independent acceptance criteria:

1. **Archive integrity** -- every file matches the SHA-256 in
   ``PACKAGE_SHA256_MANIFEST.json`` exactly.  That is what
   ``verify_package.py`` checks, and it must pass *before* a generator is run.

2. **Numerical reproduction** -- after re-running a generator, the reported
   statistics must agree, but the bytes need not.  A run on another OS writes
   different line endings; a different BLAS/NumPy/SciPy build can change the
   last bit of a float.  Neither changes a reported number.

This tool performs check 2.  It normalises CRLF/CR to LF, then compares every
non-numeric character exactly and every numeric token (or JSON number) with an
absolute/relative tolerance.  It lists every difference it tolerates and exits
non-zero only for a missing/extra file, a changed non-numeric field, a missing
or extra JSON key, or a number outside tolerance.

Usage
-----
    # Snapshot the shipped outputs before regenerating, then compare:
    python tools/verify_numbers.py --archive <snapshot-dir> --regen <regen-dir>

The default paths are the Stage-5 case:

    python tools/verify_numbers.py \
        --archive 01_Main_Experiment/results/results_v2_stage5/reproduced_archive \
        --regen   01_Main_Experiment/results/results_v2_stage5/reproduced

so the documented workflow is: copy ``reproduced/`` to
``reproduced_archive/``, run ``reproduce_stage5.py``, then run this tool.
``--archive`` may also be a directory of archived outputs from the zip.

The default tolerance is ``rtol=1e-12, atol=0``: about four orders of
magnitude above double-precision round-off for values of order 1, and many
orders below the precision at which any result in this package is reported
(three significant decimals at most).  It therefore absorbs last-bit
differences from library/BLAS versions without hiding a real change.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path

# A numeric token that is not part of an identifier or a dotted version.
# Covers 12, 0.5, .5, 1., 1e-12, -2.5E+3.
NUM = re.compile(
    r"(?<![\w.])[-+]?(?:(?:\d+\.\d*)|(?:\.\d+)|(?:\d+))"
    r"(?:[eE][-+]?\d+)?(?![\w.])"
)

BINARY_SUFFIXES = {
    ".npz", ".npy", ".zip", ".png", ".jpg", ".jpeg", ".pdf",
    ".dll", ".so", ".ico", ".gz", ".bz2", ".xz",
}


def _read_bytes(path: Path) -> bytes:
    return path.read_bytes()


def _normalise(raw: bytes) -> str:
    text = raw.decode("utf-8-sig")
    return text.replace("\r\n", "\n").replace("\r", "\n")


def _fmt(x) -> str:
    if isinstance(x, float):
        return repr(x)
    return str(x)


def _close(a, b, rtol: float, atol: float) -> bool:
    if a == b:
        return True
    return abs(a - b) <= max(rtol * max(abs(a), abs(b)), atol)


def _diff(a, b) -> tuple[float, float]:
    d = abs(a - b)
    scale = max(abs(a), abs(b))
    rel = d / scale if scale else math.inf
    return d, rel


class Report:
    def __init__(self) -> None:
        self.tolerated: list[str] = []
        self.errors: list[str] = []
        self.eol: list[str] = []
        self.files = 0
        self.json_files = 0
        self.text_files = 0
        self.binary_files = 0

    def tol(self, name: str, field: str, a, b, rtol: float) -> None:
        d, rel = _diff(a, b)
        self.tolerated.append(
            f"[TOL]  {name}: {field}  {_fmt(a)} -> {_fmt(b)}  "
            f"|d|={d:.1e} rel={rel:.1e} (rtol={rtol:g})"
        )

    def err(self, message: str) -> None:
        self.errors.append("[ERR]  " + message)


def compare_json(a, b, name: str, path: str, rep: Report,
                 rtol: float, atol: float) -> None:
    if isinstance(a, bool) or isinstance(b, bool):
        if a is not b:
            rep.err(f"{name}: {path} boolean {a!r} != {b!r}")
        return
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        if a == b:
            return
        if (isinstance(a, float) and isinstance(b, float)
                and math.isnan(a) and math.isnan(b)):
            return  # the archive uses NaN for undefined endpoints
        if _close(a, b, rtol, atol):
            rep.tol(name, path or "<root>", a, b, rtol)
        else:
            d, rel = _diff(a, b)
            rep.err(f"{name}: {path} number {_fmt(a)} != {_fmt(b)} "
                    f"(|d|={d:.1e} rel={rel:.1e} > tolerance)")
        return
    if type(a) is not type(b):
        # int/float already handled above; other type changes are structural
        rep.err(f"{name}: {path} type {type(a).__name__} != {type(b).__name__}")
        return
    if isinstance(a, dict):
        for key in sorted(a):
            if key not in b:
                rep.err(f"{name}: missing key {path + '.' if path else ''}{key}")
                continue
            compare_json(a[key], b[key], name,
                         f"{path}.{key}" if path else key, rep, rtol, atol)
        for key in sorted(b):
            if key not in a:
                rep.err(f"{name}: unexpected key {path + '.' if path else ''}{key}")
        return
    if isinstance(a, list):
        if len(a) != len(b):
            rep.err(f"{name}: {path} list length {len(a)} != {len(b)}")
        for i in range(min(len(a), len(b))):
            compare_json(a[i], b[i], name, f"{path}[{i}]", rep, rtol, atol)
        return
    if a != b:
        rep.err(f"{name}: {path} {a!r} != {b!r}")


def compare_text(name: str, archived: bytes, regenerated: bytes, rep: Report,
                 rtol: float, atol: float) -> None:
    a_text = _normalise(archived)
    b_text = _normalise(regenerated)
    a_lines = a_text.split("\n")
    b_lines = b_text.split("\n")
    if len(a_lines) != len(b_lines):
        rep.err(f"{name}: line count {len(a_lines)} != {len(b_lines)}")
    for i in range(min(len(a_lines), len(b_lines))):
        la, lb = a_lines[i], b_lines[i]
        if la == lb:
            continue
        skel_a = NUM.sub("<N>", la)
        skel_b = NUM.sub("<N>", lb)
        if skel_a != skel_b:
            rep.err(f"{name}:{i + 1}: non-numeric text differs\n"
                    f"         archive: {la[:200]}\n"
                    f"         regen  : {lb[:200]}")
            continue
        toks_a = NUM.findall(la)
        toks_b = NUM.findall(lb)
        if len(toks_a) != len(toks_b):
            rep.err(f"{name}:{i + 1}: numeric token count "
                    f"{len(toks_a)} != {len(toks_b)}")
            continue
        for j, (ta, tb) in enumerate(zip(toks_a, toks_b), 1):
            try:
                va, vb = float(ta), float(tb)
            except ValueError:  # pragma: no cover - regex guarantees floats
                continue
            if va == vb:
                continue
            if _close(va, vb, rtol, atol):
                rep.tol(name, f"line {i + 1}, number {j}", va, vb, rtol)
            else:
                d, rel = _diff(va, vb)
                rep.err(f"{name}:{i + 1}: number {j} {ta} != {tb} "
                        f"(|d|={d:.1e} rel={rel:.1e} > tolerance)")


def main(argv: list[str] | None = None) -> int:
    here = Path(__file__).resolve().parents[1]
    default_regen = (here / "01_Main_Experiment" / "results"
                     / "results_v2_stage5" / "reproduced")
    default_archive = default_regen.parent / "reproduced_archive"

    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--archive", default=str(default_archive),
                    help="archived outputs (default: %(default)s)")
    ap.add_argument("--regen", default=str(default_regen),
                    help="regenerated outputs (default: %(default)s)")
    ap.add_argument("--rtol", type=float, default=1e-12,
                    help="relative tolerance (default: %(default)s)")
    ap.add_argument("--atol", type=float, default=0.0,
                    help="absolute tolerance (default: %(default)s)")
    ap.add_argument("--quiet-eol", action="store_true",
                    help="do not list files whose only difference is line endings")
    args = ap.parse_args(argv)

    archive = Path(args.archive).resolve()
    regen = Path(args.regen).resolve()
    if not archive.is_dir():
        print(f"verify_numbers: archive directory not found: {archive}")
        print("Snapshot the shipped outputs first, e.g.:")
        print(f'  Copy-Item "{default_regen}" "{default_archive}" -Recurse')
        return 2
    if not regen.is_dir():
        print(f"verify_numbers: regenerated directory not found: {regen}")
        return 2

    print("verify_numbers: verification #2 -- numerical reproduction")
    print(f"  archive  : {archive}")
    print(f"  regen    : {regen}")
    print(f"  tolerance: rtol={args.rtol:g} atol={args.atol:g}")

    archived = sorted(p.relative_to(archive).as_posix()
                      for p in archive.rglob("*") if p.is_file())
    regenerated = sorted(p.relative_to(regen).as_posix()
                         for p in regen.rglob("*") if p.is_file())
    regen_set = set(regenerated)
    rep = Report()

    for name in archived:
        if name not in regen_set:
            rep.err(f"{name}: missing from the regenerated directory")
    for name in sorted(regen_set - set(archived)):
        rep.err(f"{name}: unexpected file in the regenerated directory")

    for name in archived:
        if name not in regen_set:
            continue
        pa = archive / name
        pb = regen / name
        raw_a, raw_b = _read_bytes(pa), _read_bytes(pb)
        rep.files += 1
        suffix = Path(name).suffix.lower()
        if suffix in BINARY_SUFFIXES:
            rep.binary_files += 1
            if raw_a != raw_b:
                rep.err(f"{name}: binary file bytes differ")
            continue
        if suffix == ".json":
            rep.json_files += 1
        else:
            rep.text_files += 1
        if raw_a == raw_b:
            continue
        # Record a line-ending-only difference, then normalise it away.
        crlf_a = b"\r\n" in raw_a
        crlf_b = b"\r\n" in raw_b
        if crlf_a != crlf_b:
            rep.eol.append(
                f"[EOL]  {name}: line endings normalised "
                f"({'CRLF' if crlf_a else 'LF'} -> {'CRLF' if crlf_b else 'LF'})"
            )
        try:
            if suffix == ".json":
                a_obj = json.loads(raw_a.decode("utf-8-sig"))
                b_obj = json.loads(raw_b.decode("utf-8-sig"))
                compare_json(a_obj, b_obj, name, "", rep,
                             args.rtol, args.atol)
            else:
                compare_text(name, raw_a, raw_b, rep, args.rtol, args.atol)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            rep.err(f"{name}: cannot parse ({exc})")

    if rep.eol and not args.quiet_eol:
        for line in rep.eol:
            print(line)
    for line in rep.tolerated:
        print(line)
    for line in rep.errors[:200]:
        print(line)
    if len(rep.errors) > 200:
        print(f"... {len(rep.errors) - 200} more error(s)")

    status = "PASS" if not rep.errors else "FAIL"
    print(
        f"RESULT: {status} - {rep.files}/{len(archived)} files "
        f"({rep.json_files} JSON, {rep.text_files} text, "
        f"{rep.binary_files} binary checked); "
        f"{len(rep.tolerated)} numeric difference(s) within tolerance; "
        f"{len(rep.eol)} file(s) with line-ending-only differences; "
        f"{len(rep.errors)} error(s)"
    )
    return 0 if not rep.errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
