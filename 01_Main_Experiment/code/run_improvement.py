"""Paired, resumable runs; one JSON per task and no modification of archived data."""
import argparse
import csv
import hashlib
import json
import os
import platform
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
import numpy as np


def execute(task):
    sid, variant, seed, out, environment, contact, audit, gate = task
    import corrections
    corrections.install(environment=environment, contact=contact)
    import sensing_gate
    sensing_gate.install(*(gate if gate is not None else (None, False)))
    from scenarios import SCENARIOS
    from proposed import Proposed
    from proposed_improved import HandoverProposed, CoordinatedProposed
    from run_experiments import ALGS
    from simulation import run_scene
    from metrics import summarize
    scn = SCENARIOS[sid]()
    kw = scn.get('params', {})
    if variant == 'handover':
        ctrl = HandoverProposed(**kw)
    elif variant == 'coordinated':
        ctrl = CoordinatedProposed(**kw)
    elif variant == 'all_primary':
        ctrl = Proposed(n_primary=scn['cfg'].n_robots, **kw)
    elif variant == 'rollout':
        from proposed_rollout import RolloutProposed
        ctrl = RolloutProposed(**kw)
    elif variant.startswith('event'):
        from proposed_event import EventProposed
        if variant == 'event':
            ctrl = EventProposed(**kw)
        elif variant == 'event_full':
            ctrl = EventProposed(n_primary=scn['cfg'].n_robots,
                                 pre_compact=1.0, pre_cmd_scale=1.0,
                                 divert_cmd_scale=1.0, **kw)
        elif variant == 'event_half':
            ctrl = EventProposed(n_primary=max(2,scn['cfg'].n_robots//2),**kw)
    else:
        ctrl = ALGS[variant](**kw)
    if audit:
        from audit_controller import AuditController
        ctrl = AuditController(ctrl)
    ctrl = sensing_gate.wrap(ctrl)
    start = time.monotonic()
    res = run_scene(scn['cfg'], scn['path'], scn['obstacles'], ctrl,
                    seed=seed, T_max=scn['T_max'], s0=scn['s0'])
    row = dict(scenario=sid, variant=variant, seed=seed,
               environment_fixed=environment, contact_fixed=contact,
               wall_seconds=time.monotonic()-start, **summarize(res))
    folder = Path(out)
    path = folder / f'{sid}__{variant.replace("/", "-")}__{seed}.json'
    if audit:
        import gzip
        trace_path=Path(str(path).replace('.json','.trace.json.gz'))
        tmp_trace=Path(str(trace_path)+'.tmp')
        with gzip.open(tmp_trace,'wt') as f:
            json.dump(ctrl.history,f)
        os.replace(tmp_trace,trace_path)
    tmp_path=Path(str(path)+'.tmp')
    tmp_path.write_text(json.dumps(row, ensure_ascii=False, indent=2, default=str))
    os.replace(tmp_path,path)
    return row


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenarios', nargs='+', default=['S3'])
    ap.add_argument('--variants', nargs='+', default=['Ours','B2','handover'])
    ap.add_argument('--seeds', nargs='+', type=int, default=[100,101,102,103,104])
    ap.add_argument('--jobs', type=int, default=8)
    ap.add_argument('--out', required=True)
    ap.add_argument('--legacy-environment', action='store_true')
    ap.add_argument('--legacy-contact', action='store_true')
    ap.add_argument('--audit', action='store_true')
    ap.add_argument('--sensing-radius', type=float, default=None, metavar='R',
                    help='obstacle detection gate radius [m] (sensing_gate.py); '
                         'default None = no gate (bit-identical to pre-gate)')
    ap.add_argument('--gate-fov', action='store_true',
                    help='also require the obstacle inside the sensor cone '
                         '(v4 semantics; breaks inertness at R->inf)')
    args = ap.parse_args()
    gate = (None, False) if args.sensing_radius is None \
        else (float(args.sensing_radius), bool(args.gate_fov))
    if 'rollout' in args.variants:
        import subprocess
        root=Path(__file__).parent
        native=root/('_rollout.dll' if __import__('sys').platform.startswith('win') else '_rollout.so')
        if not native.exists():
            raise SystemExit('build _rollout.dll first (see BUILD_ROLLOUT_WINDOWS.md)')
    folder = Path(args.out)
    folder.mkdir(parents=True, exist_ok=True)
    manifest = vars(args) | dict(python=platform.python_version(), numpy=np.__version__,
        code_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                     for p in Path(__file__).parent.glob('*.py')})
    prior=folder/'manifest.json'
    if prior.exists():
        previous=json.loads(prior.read_text())
        immutable=['proposed.py','proposed_improved.py','proposed_event.py',
                   'proposed_rollout.py','simulation.py','robots.py','sheep.py',
                   'ttc.py','config.py','scenarios.py','path.py','obstacle.py',
                   'corrections.py','sensing_gate.py']
        for name in immutable:
            if previous.get('code_sha256',{}).get(name)!=manifest['code_sha256'].get(name):
                raise RuntimeError(f'Code changed for {name}; use a new output directory.')
        for name in ['legacy_environment','legacy_contact']:
            if previous.get(name)!=manifest.get(name):
                raise RuntimeError('Correction flags changed; use a new output directory.')
        # never mix gate radii (or gate on/off) inside one output directory
        if previous.get('sensing_radius') != manifest.get('sensing_radius') or \
                previous.get('gate_fov') != manifest.get('gate_fov'):
            raise RuntimeError('Sensing gate settings changed; use a new output directory.')
    else:
        prior.write_text(json.dumps(manifest,indent=2))
    rows, tasks = [], []
    for sid in args.scenarios:
        for variant in args.variants:
            for seed in args.seeds:
                p = folder/f'{sid}__{variant.replace("/", "-")}__{seed}.json'
                if p.exists():
                    rows.append(json.loads(p.read_text()))
                else:
                    tasks.append((sid, variant, seed, str(folder),
                                  not args.legacy_environment, not args.legacy_contact,
                                  args.audit, gate))
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        fs = [pool.submit(execute, task) for task in tasks]
        for f in as_completed(fs):
            r = f.result()
            rows.append(r)
            print(f'{len(rows)} {r["scenario"]} {r["variant"]} {r["seed"]} '
                  f'C={r["collision_rate"]:.4f} d={r["min_dist"]:.3f} '
                  f'rho={r["rho_avg"]:.3f} completion={r["completion"]}', flush=True)
    if rows:
        keys = list(rows[0])
        with (folder/'detail.csv').open('w',newline='',encoding='utf-8-sig') as f:
            writer=csv.DictWriter(f,fieldnames=keys)
            writer.writeheader()
            writer.writerows(sorted(rows,key=lambda r:(r['scenario'],r['variant'],r['seed'])))
    import pandas as pd
    d = pd.DataFrame(rows)
    cols=['collision_rate','min_dist','rho_avg','max_dev','completion','success']
    print(d.groupby(['scenario','variant'])[cols].mean().to_string())
    d.groupby(['scenario','variant'])[cols].mean().to_csv(folder/'summary.csv')


if __name__ == '__main__':
    main()
