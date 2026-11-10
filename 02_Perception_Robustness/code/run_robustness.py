"""Frozen paired experiment. Resume is keyed by cell and verified source hash.
Raw results are appended and fsynced per cell; errors retain tracebacks.
"""
import os
for name in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS']:
    os.environ[name]='1'
import sys,json,time,hashlib,subprocess,platform,argparse,traceback
from pathlib import Path
from dataclasses import asdict,replace
from concurrent.futures import ProcessPoolExecutor,as_completed
import numpy as np
from v2.runner_s4 import run_cell
from v2.config_v2 import stage3_v2cfg
import fast_numpy
fast_numpy.install()

ROOT=Path(__file__).resolve().parent
FACTORS={
 'exact':{},
 'obs_pos_0p2':{'sense_err':.2},'obs_pos_0p5':{'sense_err':.5},
 'animal_pos_0p2':{'sense_animal_err':.2},'animal_pos_0p5':{'sense_animal_err':.5},
 'delay_0p2':{'sense_delay':.2},'delay_0p5':{'sense_delay':.5},
 'loss_0p05':{'sense_loss':.05},'loss_0p10':{'sense_loss':.1},
 'blackout_0p5':{'sense_blackout':.5},'blackout_1p0':{'sense_blackout':1.},
}
METHODS=['Ours_rollout','B1','B2']
SCENES=['V2S2R2','V2S3']
def source_hash():
    h=hashlib.sha256()
    for p in sorted(ROOT.rglob('*')):
        if p.suffix not in ['.py','.cpp','.so']:continue
        if '__pycache__' in p.parts:continue
        h.update(str(p.relative_to(ROOT)).encode());h.update(p.read_bytes())
    return h.hexdigest()
def config(version,factor):
    return replace(stage3_v2cfg('M1'),robust_strict=version!='archive',
                   robust_rs_safety=version=='safe',
                   c_diag_enabled=version=='archive',**FACTORS[factor])
def task(t):
    version,factor,sid,method,seed=t
    start=time.time()
    try:
        c=config(version,factor)
        row,res=run_cell(sid,method,seed,v2cfg=c,return_res=True)
        row.update(res['v2'])
        row.update(version=version,factor=factor,method=method,
                   arrival=bool(row['success']),
                   joint_completion=bool(row['success'] and row['completion']),
                   wall_s=time.time()-start,error=None)
        # Erase optional bulky diagnostics, retain scalar numerical evidence.
        for k in list(row):
            if isinstance(row[k],(list,dict,tuple)):del row[k]
        return row
    except Exception:
        return dict(version=version,factor=factor,scenario=sid,method=method,
                    seed=seed,error=traceback.format_exc(),wall_s=time.time()-start)
def key(r):return (r['version'],r['factor'],r['scenario'],r['method'],int(r['seed']))
def default(o):
    if isinstance(o,np.generic):return o.item()
    raise TypeError(type(o).__name__)
def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--out',required=True)
    ap.add_argument('--versions',nargs='+',default=['safe'])
    ap.add_argument('--factors',nargs='+',default=list(FACTORS))
    ap.add_argument('--scenes',nargs='+',default=SCENES)
    ap.add_argument('--methods',nargs='+',default=METHODS)
    ap.add_argument('--first-seed',type=int,default=3000)
    ap.add_argument('--n-seeds',type=int,default=30)
    ap.add_argument('--jobs',type=int,default=8)
    ap.add_argument('--include-decomposition',action='store_true')
    args=ap.parse_args()
    path=Path(args.out);path.parent.mkdir(parents=True,exist_ok=True)
    tasks=[(v,f,s,m,seed) for v in args.versions for f in args.factors
           for s in args.scenes for m in args.methods
           for seed in range(args.first_seed,args.first_seed+args.n_seeds)]
    if args.include_decomposition:
        tasks += [(v,'exact',s,m,seed) for v in ['archive','strict']
                  for s in args.scenes for m in args.methods
                  for seed in range(args.first_seed,args.first_seed+args.n_seeds)]
    # Fixed scheduling randomization balances short/long scenes and versions.
    order=np.random.default_rng(20261028).permutation(len(tasks))
    tasks=[tasks[int(i)] for i in order]
    digest=source_hash();manifest=path.with_suffix('.manifest.json')
    if manifest.exists():
        prior=json.loads(manifest.read_text())
        if prior['source_sha256']!=digest:raise RuntimeError('Source changed; use a new output file')
    else:
        manifest.write_text(json.dumps(dict(source_sha256=digest,args=vars(args),
            configs={v+':'+f:asdict(config(v,f)) for v in args.versions for f in args.factors},
            python=sys.version,platform=platform.platform(),numpy=np.__version__,
            compiler=subprocess.check_output(['g++','--version'],text=True).splitlines()[0],
            planned_cells=tasks),indent=2))
    done={key(r) for line in path.read_text().splitlines() if (r:=json.loads(line)) and not r.get('error')} if path.exists() else set()
    todo=[t for t in tasks if t not in done]
    print('PLANNED',len(tasks),'COMPLETE',len(done),'REMAINING',len(todo),flush=True)
    start=time.time();good=bad=0
    with path.open('a') as f,ProcessPoolExecutor(max_workers=args.jobs) as ex:
        futures={ex.submit(task,t):t for t in todo}
        for future in as_completed(futures):
            r=future.result()
            f.write(json.dumps(r,default=default)+'\n');f.flush();os.fsync(f.fileno())
            bad+=bool(r['error']);good+=not bool(r['error'])
            if r['error'] or (good+bad)%12==0:
                print('PROGRESS',good+bad,'/',len(todo),'errors',bad,'elapsed_s',round(time.time()-start),flush=True)
                if r['error']:print(r['error'],flush=True)
    print('FINISHED',good,'ERRORS',bad,'elapsed_s',round(time.time()-start),flush=True)
if __name__=='__main__':main()
