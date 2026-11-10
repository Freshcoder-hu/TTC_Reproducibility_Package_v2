"""Post-freeze geometry check; controller never receives the release schedule."""
import argparse
import json
import hashlib
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor,as_completed
import pandas as pd
from run_improvement import execute

def run(task):
    interval,variant,seed,out=task
    from scenarios import SCENARIOS,scenario_3
    from path import straight_path
    def scene():
        s=scene_base=scenario_3()
        for i,o in enumerate(s['obstacles']):
            o.t_spawn=2.+i*interval
            o.start[0]=8.+o.t_spawn+50.
            o.pos=o.start.copy()
        s['path']=straight_path(0,110+2*(interval-28),halfwidth=6.)
        s['T_max']=240.
        s['name']=f'S3_interval_{interval:g}s'
        return s
    sid=f'S3I{interval:g}'
    SCENARIOS[sid]=scene
    return execute((sid,variant,seed,out,True,True,False))

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--jobs',type=int,default=3)
    ap.add_argument('--out',default='results_interval_robustness');args=ap.parse_args()
    folder=Path(args.out);folder.mkdir(exist_ok=True)
    protocol=dict(intervals=[22,36],seeds=list(range(400,405)),variants=['Ours','B2','rollout'],
                  model_freeze=json.loads(Path('MODEL_FREEZE.json').read_text()))
    (folder/'manifest.json').write_text(json.dumps(protocol,indent=2))
    rows=[];tasks=[]
    for interval in protocol['intervals']:
        for variant in protocol['variants']:
            for seed in protocol['seeds']:
                p=folder/f'S3I{interval}__{variant}__{seed}.json'
                if p.exists():rows.append(json.loads(p.read_text()))
                else:tasks.append((interval,variant,seed,str(folder)))
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        futures=[pool.submit(run,t) for t in tasks]
        for f in as_completed(futures):
            r=f.result();rows.append(r)
            print(len(rows),r['scenario'],r['variant'],r['seed'],r['collision_rate'],flush=True)
    d=pd.DataFrame(rows);d.to_csv(folder/'detail.csv',index=False,encoding='utf-8-sig')
    cols=['collision_rate','min_dist','rho_avg','max_dev','completion','success']
    s=d.groupby(['scenario','variant'])[cols].mean();s.to_csv(folder/'summary.csv');print(s.to_string())
