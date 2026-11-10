"""Targeted low-allocation timing checks of two observed peak cases.
Retain the original panel; do not pool or replace its samples. No GC disabling.
"""
import os
for name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[name]='1'
import csv,gc,hashlib,json,sys,time
from pathlib import Path
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'code'))
from run_robustness import config,source_hash
from v2.runner_s4 import run_cell,ALGS
from v2.scenarios_v2 import resolve_scene
from v2.controller_v2 import make_controller,unpack
from v2.robust_sensing import RobustSensorium
from v2.fleet_v2 import RobotFleetV2
from v2.rs_safety import RSSafety
from v2.feasibility_v2 import FeasibilityLayer

OUT=ROOT/'analysis/online_measurement/overhead_checks'
PHASES=['sense_update','sense_publish','controller','fleet_total','feasibility','rs_safety','rollout_native','rs_native']

def main():
    OUT.mkdir(exist_ok=True)
    os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
    results=[]
    rows=[json.loads(x) for x in (ROOT/'results/formal.jsonl').read_text().splitlines()]
    for method in ('B2','Ours_rollout'):
        scenario,factor,seed='V2S3','blackout_1p0',3000
        cfg=config('safe',factor)
        ctrl=make_controller(method,ALGS,cfg,dict(resolve_scene(scenario).get('params',{})))
        wall=np.zeros((10000,len(PHASES)),dtype=np.int64);cpu=np.zeros_like(wall)
        gct=np.zeros(10000,dtype=np.int64);gcc=np.zeros(10000,dtype=np.int64)
        generations=np.zeros((10000,3),dtype=np.int64)
        step=-1;gc_start=(0,0);n=0
        def timed(index,fn,*args,**kw):
            w0,c0=time.perf_counter_ns(),time.process_time_ns()
            out=fn(*args,**kw)
            c1,w1=time.process_time_ns(),time.perf_counter_ns()
            if step>=0:
                wall[step,index]+=w1-w0;cpu[step,index]+=c1-c0
            return out
        def callback(phase,info):
            nonlocal gc_start
            if phase=='start':gc_start=(time.perf_counter_ns(),time.process_time_ns())
            elif step>=0:
                gct[step]+=time.perf_counter_ns()-gc_start[0]
                gcc[step]+=time.process_time_ns()-gc_start[1]
                generations[step,info['generation']]+=1
        original_update=RobustSensorium.update
        original_publish=RobustSensorium.publish
        original_fleet=RobotFleetV2.step
        original_feas=FeasibilityLayer.filter
        original_rs=RSSafety.filter
        original_rs_init=RSSafety.__init__
        original_ctrl=ctrl.step
        def update(obj,t,s,*args,**kw):
            nonlocal step,n
            step=int(s);n=step+1
            return timed(0,original_update,obj,t,s,*args,**kw)
        def rs_init(obj,*args,**kw):
            original_rs_init(obj,*args,**kw)
            native=obj.fun
            obj.fun=lambda *a:timed(7,native,*a)
        ctrl.step=lambda *a,**kw:timed(2,original_ctrl,*a,**kw)
        core,_,_=unpack(ctrl)
        if method=='Ours_rollout':
            native=core._evaluate
            core._evaluate=lambda *a:timed(6,native,*a)
        gc.callbacks.append(callback)
        try:
            with patch.object(RobustSensorium,'update',update), \
                 patch.object(RobustSensorium,'publish',lambda obj,*a,**kw:timed(1,original_publish,obj,*a,**kw)), \
                 patch.object(RobotFleetV2,'step',lambda obj,*a,**kw:timed(3,original_fleet,obj,*a,**kw)), \
                 patch.object(FeasibilityLayer,'filter',lambda obj,*a,**kw:timed(4,original_feas,obj,*a,**kw)), \
                 patch.object(RSSafety,'filter',lambda obj,*a,**kw:timed(5,original_rs,obj,*a,**kw)), \
                 patch.object(RSSafety,'__init__',rs_init):
                row,res=run_cell(scenario,method,seed,v2cfg=cfg,controller=ctrl,
                                 return_res=True,record_states=False)
        finally:gc.callbacks.remove(callback)
        row.update(res['v2'])
        expected=next(r for r in rows if r['version']=='safe' and r['scenario']==scenario
            and r['method']==method and r['factor']==factor and r['seed']==seed)
        for k in ('collision_rate','rho_avg','success','completion','min_surface_clearance_rs',
                  'min_surface_clearance_so','any_contact_rs','any_contact_so','n_plans'):
            if k in row and k in expected:
                a,b=row[k],expected[k]
                assert a==b or (isinstance(a,(float,np.floating)) and isinstance(b,(float,np.floating)) and np.isnan(a) and np.isnan(b)),(method,k)
        wall,cpu,gct,gcc,generations=(x[:n] for x in (wall,cpu,gct,gcc,generations))
        work=wall[:,:4].sum(axis=1)/1e6;workcpu=cpu[:,:4].sum(axis=1)/1e6
        cycles=[float(work[i:i+2].sum()) for i in range(20,n-1,2)]
        worst=int(np.argmax(work))
        item=dict(method=method,scenario=scenario,factor=factor,seed=seed,
            performance_scalar_parity=True,dense_state_retention=False,
            diagnostic_event_arrays=False,garbage_collection_enabled=True,
            steps=n,step_over_50ms=int(np.sum(work[20:]>50)),
            step_p99_ms=float(np.quantile(work[20:],.99)),step_max_ms=float(work.max()),
            cycle_max_ms=max(cycles),cycle_over_100ms=sum(x>100 for x in cycles),
            worst_step=worst,worst_t=worst*.05,worst_step_cpu_ms=float(workcpu[worst]),
            worst_step_gc_ms=float(gct[worst])/1e6,
            max_observed_gc_pause_ms=float(gct.max())/1e6,
            total_gc_count_by_generation=generations.sum(axis=0).tolist(),
            worst_step_phases_ms={p:float(wall[worst,j])/1e6 for j,p in enumerate(PHASES)},
            native_rs_max_ms=float(wall[:,7].max())/1e6,
            native_rollout_max_ms=float(wall[:,6].max())/1e6)
        results.append(item)
        np.savez_compressed(OUT/(method+'.npz'),wall_ns=wall,cpu_ns=cpu,gc_wall_ns=gct,
                            gc_cpu_ns=gcc,gc_generations=generations)
        print(json.dumps(item),flush=True)
    frozen=json.loads((ROOT/'results/formal.manifest.json').read_text())['source_sha256']
    assert source_hash()==frozen
    summary=dict(status='complete',targeted_replays=2,not_pooled_with_prespecified_timing_panel=True,
        frozen_source_unchanged=True,results=results,
        limits=['Targeted followup of observed peaks, not an independent worst-case bound',
                'Different GC/retention conditions can change timing; original panel is retained',
                'Even these measurements include nonzero recorder overhead'])
    (OUT/'summary.json').write_text(json.dumps(summary,indent=2))

if __name__=='__main__':main()
