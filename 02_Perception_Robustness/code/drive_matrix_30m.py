"""Driver for the R = 30 m main matrix: 8 methods x 10 scenarios x 30 seeds.

One subprocess per method, so a crash loses at most one batch (each batch writes
`_parts/<alg>/detail.csv`).  Appends a line to `_progress.txt` after every batch
and is resume-safe: a batch whose `detail.csv` already holds the full 240-row*
complement is skipped.

* 10 scenarios x 30 seeds = 300 rows per method, 2400 runs in total.

Usage:
    python drive_matrix_30m.py                 # full 30 m matrix
    python drive_matrix_30m.py --radius 10000 --out results_cea_matrix_inf
    python drive_matrix_30m.py --only B1 B2    # subset of methods
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).parent
ALGS = ["Ours_rollout", "B1", "B2", "VH-Path", "VH-Danger", "VH-Danger-path",
        "B-Outmost", "B-PCBF"]
SCEN = ["S1", "S1N", "S2", "S2R3", "S2R2", "S2R1", "S3", "S4S", "S4M", "S4L"]
ENV = dict(os.environ, OMP_NUM_THREADS="1", MKL_NUM_THREADS="1",
           OPENBLAS_NUM_THREADS="1", NUMEXPR_NUM_THREADS="1",
           PYTHONIOENCODING="utf-8", PYTHONUTF8="1")


def stamp():
    return time.strftime("%H:%M:%S")


def nrows(p):
    if not p.exists():
        return 0
    with open(p, newline="", encoding="utf-8-sig") as f:
        return max(0, sum(1 for _ in f) - 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="results_cea_matrix_30m")
    ap.add_argument("--radius", type=float, default=30.0)
    ap.add_argument("--seeds", type=int, default=30)
    ap.add_argument("--seed-offset", type=int, default=200)
    ap.add_argument("--jobs", type=int, default=12)
    ap.add_argument("--only", nargs="+", default=ALGS)
    ap.add_argument("--fov", action="store_true")
    args = ap.parse_args()

    out = ROOT / args.out
    out.mkdir(parents=True, exist_ok=True)
    prog = out / "_progress.txt"
    log = out / "matrix_log.txt"
    want = len(SCEN) * args.seeds
    t_all = time.time()
    with prog.open("a", encoding="utf-8") as f:
        f.write(f"[{stamp()}] MATRIX START R={args.radius:g} m "
                f"scenarios={len(SCEN)} seeds={args.seeds}"
                f"@{args.seed_offset} jobs={args.jobs} methods={len(args.only)}"
                f" fov={args.fov} expected={want * len(args.only)} runs\n")

    done, skipped, failed = 0, 0, []
    for i, alg in enumerate(args.only, start=1):
        dst = out / "_parts" / alg
        dst.mkdir(parents=True, exist_ok=True)
        detail = dst / "detail.csv"
        meta_p = dst / "_run_meta.json"
        meta = dict(radius_m=float(args.radius), gate_fov=bool(args.fov),
                    scenarios=SCEN, seeds=args.seeds,
                    seed_offset=args.seed_offset, alg=alg,
                    driver="drive_matrix_30m.py",
                    python=sys.version.split()[0],
                    env={k: ENV[k] for k in ("OMP_NUM_THREADS",
                                             "MKL_NUM_THREADS",
                                             "OPENBLAS_NUM_THREADS")},
                    started=time.strftime("%Y-%m-%d %H:%M:%S"))
        if meta_p.exists():
            old = json.loads(meta_p.read_text(encoding="utf-8"))
            if (float(old.get("radius_m", -1)) != float(args.radius)
                    or bool(old.get("gate_fov", False)) != bool(args.fov)
                    or old.get("seed_offset") != args.seed_offset):
                raise SystemExit(
                    f"REFUSING to mix settings in {dst}: on disk "
                    f"R={old.get('radius_m')} fov={old.get('gate_fov')} "
                    f"seed_offset={old.get('seed_offset')}")
        else:
            meta_p.write_text(json.dumps(meta, indent=2), encoding="utf-8")

        have = nrows(detail)
        if have >= want:
            skipped += 1
            with prog.open("a", encoding="utf-8") as f:
                f.write(f"[{stamp()}] batch {i}/{len(args.only)} {alg} "
                        f"SKIP (already {have}/{want} rows)\n")
            continue

        cmd = [sys.executable, "-X", "utf8", "-u", "run_revised_experiments.py",
               "--scenarios", *SCEN, "--seeds", str(args.seeds),
               "--seed-offset", str(args.seed_offset), "--algs", alg,
               "--jobs", str(args.jobs), "--out", str(dst),
               "--sensing-radius", str(args.radius)]
        if args.fov:
            cmd.append("--gate-fov")
        rc, el = None, None
        for attempt in (1, 2):
            t0 = time.time()
            with prog.open("a", encoding="utf-8") as f:
                f.write(f"[{stamp()}] batch {i}/{len(args.only)} {alg} START"
                        f" (attempt {attempt})\n")
            with log.open("a", encoding="utf-8") as lf:
                lf.write(f"\n=== {time.strftime('%Y-%m-%d %H:%M:%S')} "
                         f"{alg} R={args.radius:g} attempt={attempt} ===\n")
                lf.flush()
                rc = subprocess.run(cmd, cwd=ROOT, env=ENV,
                                    stdout=lf, stderr=subprocess.STDOUT,
                                    encoding="utf-8", errors="replace").returncode
            el = time.time() - t0
            if rc == 0:
                break
            with prog.open("a", encoding="utf-8") as f:
                f.write(f"[{stamp()}] batch {i}/{len(args.only)} {alg} "
                        f"FAILED rc={rc} wall={el:.1f}s\n")
            time.sleep(5)
        rows = nrows(detail)
        msg = (f"[{stamp()}] batch {i}/{len(args.only)} {alg} "
               f"{'DONE' if rc == 0 else 'ERROR'} rc={rc} rows={rows}/{want} "
               f"wall={el:.1f}s ({el / 60:.1f} min)")
        print(msg, flush=True)
        with prog.open("a", encoding="utf-8") as f:
            f.write(msg + "\n")
        if rc != 0:
            failed.append(alg)
        done += 1

    total = time.time() - t_all
    tail = (f"[{stamp()}] MATRIX END ran={done} skipped={skipped} "
            f"failed={failed} total={total / 60:.1f} min")
    print(tail, flush=True)
    with prog.open("a", encoding="utf-8") as f:
        f.write(tail + "\n")
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
