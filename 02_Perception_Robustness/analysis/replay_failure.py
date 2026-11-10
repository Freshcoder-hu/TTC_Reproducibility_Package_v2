"""Replay a fixed observed failure with dense, read-only state recording.
Not part of the primary inference; confirms the failure and its temporal context.
"""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
from pathlib import Path
import sys,json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'code'))
from run_robustness import config
from v2.runner_s4 import run_cell
OUT=ROOT/'analysis/failure_replay';OUT.mkdir(exist_ok=True)
rows=[]
for f in ['exact','blackout_1p0']:
 row,res=run_cell('V2S3','Ours_rollout',3009,v2cfg=config('safe',f),return_res=True,record_states=True)
 tr=res['trace_states']
 t=np.array([x[0] for x in tr]);sp=np.stack([x[3] for x in tr]);rp=np.stack([x[6] for x in tr])
 np.savez_compressed(OUT/(f+'.npz'),t=t+.05,sheep_pos=sp,robot_pos=rp,
      robot_speed=np.stack([x[8] for x in tr]),robot_heading=np.stack([x[7] for x in tr]))
 sensed=res['sense_log'];st=np.array([x['t'] for x in sensed]);sn=np.array([x['n_animals'] for x in sensed])
 ni=np.clip(np.searchsorted(st,t+1e-8,side='right')-1,0,len(st)-1)
 minimum=[]
 for k in range(len(t)):
  r=rp[k,:,None,:]-sp[k,None,:,:]
  if k:
   old=rp[k-1,:,None,:]-sp[k-1,None,:,:];d=r-old
   ss=(d*d).sum(axis=-1);u=np.clip(-(old*d).sum(axis=-1)/np.maximum(ss,1e-30),0,1)
   r=old+u[...,None]*d
  minimum.append(float(np.linalg.norm(r,axis=-1).min()-.8))
 frame=pd.DataFrame(dict(t=t+.05,rs_min_swept=minimum,n_animals=sn[ni]))
 frame.to_csv(OUT/(f+'.csv'),index=False)
 row.update(factor=f,method='Ours_rollout',version='safe',seed=3009,scenario='V2S3')
 rows.append({k:row.get(k) for k in ['factor','scenario','method','seed','collision_rate','completion','success','min_surface_clearance_rs','any_contact_rs']})
 with (ROOT/'results/formal.jsonl').open() as fp:
  formal=[json.loads(l) for l in fp]
 match=[x for x in formal if x['version']=='safe' and x['factor']==f and x['scenario']=='V2S3' and x['method']=='Ours_rollout' and x['seed']==3009]
 if match:
  for k in ['collision_rate','completion','success','min_surface_clearance_rs','any_contact_rs']:
   assert row[k]==match[0][k],(f,k,row[k],match[0][k])
 print(f,rows[-1],flush=True)
(OUT/'summary.json').write_text(json.dumps(rows,indent=2))
frames={f:pd.read_csv(OUT/(f+'.csv')) for f in ['exact','blackout_1p0']}
f=frames['blackout_1p0'];tc=float(f.loc[f.rs_min_swept.idxmin(),'t'])
fig,ax=plt.subplots(2,1,figsize=(10.5,6.2))
for a in ax:
 for name,c,label in [('exact','#137d87','Exact'),('blackout_1p0','#b75f39','1 s shared animal outage')]:
  z=frames[name];a.plot(z.t,z.rs_min_swept,color=c,label=label,lw=1)
 a.axhline(0,color='#9a4949',lw=1);a.grid(axis='y',color='#e0e6e8');a.set_ylabel('Minimum robot-animal clearance (m)')
 for begin,end in zip(f.t[(f.n_animals==0)&(f.n_animals.shift(fill_value=36)>0)],f.t[(f.n_animals>0)&(f.n_animals.shift(fill_value=36)==0)]):
  a.axvspan(begin,end,color='#b75f39',alpha=.15)
ax[0].set_title('V2S3 / Ours / seed 3009: dense replay, no controller changes')
ax[0].legend(frameon=False);ax[1].set_xlim(tc-4,tc+4);ax[1].set_ylim(min(f.rs_min_swept.min()-.05,-.1),1.)
ax[1].set_xlabel('Simulation time (s); shaded = no animal observations')
fig.tight_layout();fig.savefig(OUT/'replay.png',dpi=180);plt.close(fig)
