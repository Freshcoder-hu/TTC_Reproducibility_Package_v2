#!/usr/bin/env python3
"""Run the ported figure scripts and grade each output against the shipped PDF.

For every figure the submission package ships (``04_Quick_Access/
Submitted_Figures/Figure_1..9.pdf`` + ``.png``) this harness

1. runs the script that draws it, capturing the exit code, the console output
   and the wall-clock runtime;
2. compares the produced ``.pdf``/``.png`` with the shipped pair and grades it:

   ``BYTE-IDENTICAL``       same SHA-256 (the whole file, both formats)
   ``PIXEL-IDENTICAL``      different bytes, but every pixel is equal when both
                            PDFs are rasterised at the same DPI -- normally the
                            only byte difference is the ``/CreationDate`` string
   ``VISUALLY-EQUIVALENT``  same content on a differently sized canvas (the
                            SI set was drawn at the earlier 190 mm width);
                            graded by rescaling to the shipped pixel grid and
                            requiring < 5 % differing pixels
   ``DIFFERENT``            something else changed
   ``FAILS``                the script exited non-zero or wrote no file

The ``.png`` comparison needs only Pillow (a matplotlib dependency); the PDF
raster comparison additionally uses PyMuPDF when it is installed.

Usage (from the repository root)::

    python 07_Figure_Source/compare_figures.py
    python 07_Figure_Source/compare_figures.py --no-run      # grade build/
    python 07_Figure_Source/compare_figures.py --si-ref DIR  # also grade S1-S4
    python 07_Figure_Source/compare_figures.py --ga-ref FILE # also grade the GA

``build/`` is covered by ``.gitignore``; nothing is written into the shipped
tree.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
SUB = ROOT / "04_Quick_Access" / "Submitted_Figures"

BODY = [("Figure_%d" % i, "Fig%d" % i) for i in range(1, 9)] + [("Figure_9", "Fig9")]

SCRIPTS = [
    ("fig_body.py", ["fig_body.py"]),
    ("fig_loss.py", ["fig_loss.py"]),
    ("fig_si.py", ["fig_si.py"]),
    ("make_graphical_abstract.py", ["make_graphical_abstract.py"]),
]


def sha(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _png_array(path: pathlib.Path):
    import numpy as np
    from PIL import Image
    return np.asarray(Image.open(path).convert("RGB"))


def _pdf_array(path: pathlib.Path):
    try:
        import pymupdf
    except ImportError:
        try:
            import fitz as pymupdf  # older name
        except ImportError:
            return None
    import numpy as np
    doc = pymupdf.open(str(path))
    pix = doc[0].get_pixmap(dpi=300, colorspace=pymupdf.csRGB, alpha=False)
    arr = np.frombuffer(pix.samples, dtype=np.uint8).reshape(
        pix.height, pix.width, pix.n)
    doc.close()
    return arr


def _diff(a, b):
    import numpy as np
    if a.shape != b.shape:
        return None
    d = np.abs(a.astype(np.int16) - b.astype(np.int16))
    return dict(diff_px=int((d.max(axis=2) > 8).sum()),
                total_px=int(a.shape[0] * a.shape[1]), maxdiff=int(d.max()))


def _rescaled_diff(a, b):
    """Diff after resampling *a* onto *b*'s pixel grid (different page sizes)."""
    import numpy as np
    from PIL import Image
    im = Image.fromarray(a).resize((b.shape[1], b.shape[0]), Image.LANCZOS)
    d = np.abs(np.asarray(im).astype(np.int16) - b.astype(np.int16))
    return dict(diff_px=int((d.max(axis=2) > 8).sum()),
                total_px=int(b.shape[0] * b.shape[1]), maxdiff=int(d.max()))


def grade(mine_pdf, mine_png, ref_pdf, ref_png):
    """Return (verdict, detail dict) for one produced figure."""
    import numpy as np

    if not mine_pdf.is_file() and not mine_png.is_file():
        return "FAILS", {"why": "no output file"}

    same_pdf = mine_pdf.is_file() and ref_pdf.is_file() and sha(mine_pdf) == sha(ref_pdf)
    same_png = mine_png.is_file() and ref_png.is_file() and sha(mine_png) == sha(ref_png)
    if same_pdf and same_png:
        return "BYTE-IDENTICAL", {"files": "pdf+png"}
    if same_png and not same_pdf:
        return "PIXEL-IDENTICAL", {"files": "png byte-identical, pdf differs"}

    detail = {}
    if not (mine_png.is_file() and ref_png.is_file()):
        detail["png"] = "no png pair"
        a = b = None
    else:
        a, b = _png_array(mine_png), _png_array(ref_png)
    d = _diff(a, b) if a is not None else None
    if a is not None and d is not None:
        detail.update({"png_diff_px": d["diff_px"], "png_total_px": d["total_px"]})
        if d["diff_px"] == 0:
            return "PIXEL-IDENTICAL", dict(detail, files="png (byte-different pdf)")
    elif a is not None:
        d2 = _rescaled_diff(a, b)
        detail.update({"png_rescaled_diff_px": d2["diff_px"],
                       "png_rescaled_frac_pct": round(
                           100.0 * d2["diff_px"] / d2["total_px"], 3),
                       "size_mine": list(a.shape[:2]), "size_ref": list(b.shape[:2])})
        if d2["diff_px"] / d2["total_px"] < 0.05:
            return "VISUALLY-EQUIVALENT", detail

    if not (mine_pdf.is_file() and ref_pdf.is_file()):
        return "DIFFERENT", dict(detail, why="no pdf pair to compare")
    pa, pb = _pdf_array(mine_pdf), _pdf_array(ref_pdf)
    if pa is not None and pb is not None:
        dp = _diff(pa, pb)
        if dp is not None:
            detail.update({"pdf_diff_px": dp["diff_px"]})
            if dp["diff_px"] == 0:
                return "PIXEL-IDENTICAL", dict(detail, files="pdf raster 300 dpi")
        else:
            dp2 = _rescaled_diff(pa, pb)
            detail.update({"pdf_rescaled_frac_pct": round(
                100.0 * dp2["diff_px"] / dp2["total_px"], 3)})
            if dp2["diff_px"] / dp2["total_px"] < 0.05:
                return "VISUALLY-EQUIVALENT", detail
    return "DIFFERENT", detail


def pdf_metadata_only(mine, ref) -> bool:
    """True when the two PDFs differ on the /CreationDate line and nowhere else."""
    a = mine.read_bytes().split(b"\n")
    b = ref.read_bytes().split(b"\n")
    if len(a) != len(b):
        return False
    return sum(1 for x, y in zip(a, b) if x != y) <= 2


def run_scripts(out: pathlib.Path, only=None):
    """Run the four scripts into *out* the way their own defaults lay it out."""
    out.mkdir(parents=True, exist_ok=True)
    (out / "body").mkdir(exist_ok=True)
    (out / "si").mkdir(exist_ok=True)
    results = []
    for name, argv in SCRIPTS:
        if only and name not in only:
            continue
        cmd = [sys.executable, str(HERE / argv[0])]
        if name == "make_graphical_abstract.py":
            cmd += ["--out-dir", str(out)]
        elif name == "fig_si.py":
            cmd += ["--out", str(out / "si")]
        else:
            cmd += ["--out", str(out / "body")]
        t0 = time.perf_counter()
        proc = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True)
        dt = time.perf_counter() - t0
        tail = (proc.stdout or "").strip().splitlines()
        results.append(dict(script=name, exit=proc.returncode,
                            seconds=round(dt, 2),
                            stdout_tail=tail[-1] if tail else "",
                            stderr_tail=((proc.stderr or "").strip().splitlines()
                                         or [""])[-1]))
        print("$ python 07_Figure_Source/%s" % argv[0])
        for line in (proc.stdout or "").rstrip().splitlines():
            print("   ", line)
        if proc.stderr.strip():
            for line in proc.stderr.strip().splitlines():
                print("  !", line)
        print("    exit %d in %.1f s\n" % (proc.returncode, dt))
    return results


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", default=str(HERE / "build"),
                    help="build root for the rebuilt figures; the body set goes "
                         "to OUT/body, the supplementary set to OUT/si, the "
                         "graphical abstract to OUT")
    ap.add_argument("--no-run", action="store_true",
                    help="grade the files already in --out")
    ap.add_argument("--only", default=None,
                    help="comma list of scripts to run, e.g. fig_body.py")
    ap.add_argument("--si-ref", default=None,
                    help="directory holding the shipped Figure_S1..S4.pdf")
    ap.add_argument("--ga-ref", default=None,
                    help="path to the shipped graphical_abstract.pdf")
    ap.add_argument("--json", default=None,
                    help="write the graded table as JSON to this path")
    args = ap.parse_args(argv)

    out = pathlib.Path(args.out)
    body_dir, si_dir = out / "body", out / "si"
    only = set(v.strip() for v in args.only.split(",")) if args.only else None
    runs = [] if args.no_run else run_scripts(out, only)

    rows = []
    worst = 0
    for label, stem in BODY:
        verdict, detail = grade(body_dir / (stem + ".pdf"), body_dir / (stem + ".png"),
                                SUB / (label + ".pdf"), SUB / (label + ".png"))
        extra = ""
        mine_pdf, ref_pdf = body_dir / (stem + ".pdf"), SUB / (label + ".pdf")
        if verdict == "PIXEL-IDENTICAL" and mine_pdf.is_file() and ref_pdf.is_file() \
                and pdf_metadata_only(mine_pdf, ref_pdf):
            extra = "pdf differs only in /CreationDate"
        rows.append(dict(figure=label, produced=stem + ".pdf", verdict=verdict,
                         detail=detail, note=extra))
        if verdict == "FAILS":
            worst = 2

    if args.si_ref:
        for i in range(1, 5):
            ref = pathlib.Path(args.si_ref) / ("Figure_S%d.pdf" % i)
            verdict, detail = grade(si_dir / ("FigS%d.pdf" % i), si_dir / ("FigS%d.png" % i),
                                    ref, ref.with_suffix(".png"))
            rows.append(dict(figure="Figure_S%d" % i, produced="FigS%d.pdf" % i,
                             verdict=verdict, detail=detail, note="supplementary"))
    if args.ga_ref:
        ref = pathlib.Path(args.ga_ref)
        verdict, detail = grade(out / "graphical_abstract.pdf",
                                out / "graphical_abstract.png",
                                ref, ref.with_suffix(".png"))
        rows.append(dict(figure="graphical_abstract", produced="graphical_abstract.pdf",
                         verdict=verdict, detail=detail, note="not a body figure"))

    print("%-18s %-8s %-20s %s" % ("figure", "file", "verdict", "detail"))
    print("-" * 96)
    for r in rows:
        print("%-18s %-8s %-20s %s" % (r["figure"], r["produced"], r["verdict"],
                                       json.dumps(r["detail"], sort_keys=True)))
        if r["note"]:
            print("%-18s %-8s %-20s %s" % ("", "", "", r["note"]))
    print()
    if runs:
        print("%-28s %-6s %-9s %s" % ("script", "exit", "runtime", "last line"))
        for r in runs:
            print("%-28s %-6d %-9s %s" % (r["script"], r["exit"],
                                          "%.1f s" % r["seconds"], r["stdout_tail"]))

    counts = {}
    for r in rows:
        counts[r["verdict"]] = counts.get(r["verdict"], 0) + 1
    print("\nRESULT:", ", ".join("%s=%d" % kv for kv in sorted(counts.items())))

    if args.json:
        pathlib.Path(args.json).write_text(
            json.dumps(dict(runs=runs, rows=rows), indent=2), encoding="utf-8")
    return worst


if __name__ == "__main__":
    sys.exit(main())
