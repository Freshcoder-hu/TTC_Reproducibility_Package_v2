"""Paired inference, completion accounting, and static publication figures.
Run only after all 2340 cells have completed. Every summary keeps all seeds.
"""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
from pathlib import Path
import sys,json,csv,hashlib
import numpy as np
import pandas as pd
from scipy.stats import binomtest
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'code'))
from run_robustness import FACTORS,source_hash
import fast_numpy
np.clip=fast_numpy._original_clip
np.linalg.norm=fast_numpy._original_norm
OUT=ROOT/'analysis';FIG=OUT/'figures';FIG.mkdir(exist_ok=True)
LABELS=['Exact','Obs .2','Obs .5','Animal .2','Animal .5',
        'Delay .2','Delay .5','Loss 5%','Loss 10%','Out .5','Out 1']
METHODS=['Ours_rollout','B1','B2'];COLORS=['#137d87','#d38237','#6c6c91']

def clean(v):
 if isinstance(v,dict):return {str(k):clean(x) for k,x in v.items()}
 if isinstance(v,(list,tuple)):return [clean(x) for x in v]
 if isinstance(v,np.generic):v=v.item()
 if isinstance(v,float) and not np.isfinite(v):return None
 return v

def dump(path,v):path.write_text(json.dumps(clean(v),ensure_ascii=False,indent=2,allow_nan=False))
def bootstrap(d,seed=981,n=10000):
 d=np.asarray(d,dtype=float);rng=np.random.default_rng(seed)
 v=d[rng.integers(0,len(d),(n,len(d)))].mean(axis=1)
 return [float(x) for x in np.quantile(v,[.025,.975])]
def signflip(d,seed=827,n=100000):
 d=np.asarray(d,float);d=d[np.abs(d)>1e-15]
 if not len(d):return 1.
 obs=abs(d.sum())
 if len(d)<=17:
  bits=np.arange(2**len(d),dtype=np.uint64)[:,None]
  signs=((bits>>np.arange(len(d),dtype=np.uint64))&1).astype(float)*2-1
  return float(np.mean(np.abs(signs@d)>=obs-1e-12))
 rng=np.random.default_rng(seed);exceed=0
 for k in range(0,n,10000):
  signs=rng.integers(0,2,(min(10000,n-k),len(d)))*2-1
  exceed+=int(np.sum(np.abs(signs@d)>=obs-1e-12))
 return (exceed+1)/(n+1)
def holm(p):
 p=np.array(p,float);order=np.argsort(p);adjust=np.maximum.accumulate((len(p)-np.arange(len(p)))*p[order])
 out=np.empty_like(p);out[order]=np.minimum(adjust,1);return out

def main():
 rows=[json.loads(x) for x in (ROOT/'results/formal.jsonl').read_text().splitlines()]
 errors=[x for x in rows if x.get('error')]
 manifest=json.loads((ROOT/'results/formal.manifest.json').read_text())
 assert not errors,errors[:1]
 assert source_hash()==manifest['source_sha256'],'Source changed during evaluation'
 df=pd.DataFrame(rows)
 df['posthoc_avoidability_valid']=df.version=='archive'
 keys=['version','factor','scenario','method','seed']
 assert len(df)==2340 and not df.duplicated(keys).any(),len(df)
 assert sorted(df.seed.unique())==list(range(3000,3030))
 assert ((df.collision_rate>=0)&(df.collision_rate<=1)).all()
 main_df=df[df.version=='safe'].copy()
 assert len(main_df)==1980
 sizes=main_df.groupby(['factor','scenario','method']).size()
 assert len(sizes)==66 and (sizes==30).all()
 assert (df[df.version!='archive'].sense_animal_fallback_ticks==0).all()
 assert not df[df.version!='archive'].truth_in_execution.fillna(True).any()
 df.sort_values(keys).to_csv(OUT/'all_runs.csv',index=False,encoding='utf-8-sig')
 records=[]
 for k,g in df.groupby(['version','factor','scenario','method'],sort=False):
  records.append(dict(zip(['version','factor','scenario','method'],k),n=len(g),
   C=g.collision_rate.mean(),C_sd=g.collision_rate.std(),C_ci=bootstrap(g.collision_rate),
   rho=g.rho_avg.mean(),arrival=int(g.arrival.sum()),joint=int(g.joint_completion.sum()),
   rs_contact_runs=int(g.any_contact_rs.sum()),ro_contact_runs=int(g.any_contact_ro.sum()),
   rr_contact_runs=int(g.any_contact_rr.sum()),
   so_clearance_mean=g.min_surface_clearance_so.mean(),so_clearance_worst=g.min_surface_clearance_so.min(),
   rs_clearance_mean=g.min_surface_clearance_rs.mean(),rs_clearance_worst=g.min_surface_clearance_rs.min(),
   rs_events=int(g.n_contact_events_rs.sum()),violations_mean=g.violation_rate.mean(),
   robot_limit_violations=int(g[['violation_R1','violation_R2','violation_R3','violation_R4_src','violation_lat_src']].sum().sum()),
   animal_observed_mean=g.sense_animal_obs_mean.mean(),animal_empty_ticks=g.get('sense_empty_animal_ticks',pd.Series(dtype=float)).sum(),
   age_obstacle_mean=g.sense_track_age_mean.mean(),age_animal_mean=g.get('sense_animal_age_mean',pd.Series(dtype=float)).mean(),
   no_safe_candidates=g.get('rs_no_safe_candidate',pd.Series(dtype=float)).sum(),
   wall_s=g.wall_s.sum()))
 summary=pd.DataFrame(records)
 summary.to_csv(OUT/'summary.csv',index=False,encoding='utf-8-sig')
 pairs=[]
 for si,scene in enumerate(['V2S2R2','V2S3']):
  for fi,factor in enumerate(FACTORS):
   g=main_df[(main_df.scenario==scene)&(main_df.factor==factor)]
   for bi,base in enumerate(['B1','B2']):
    p=g.pivot(index='seed',columns='method',values='collision_rate').sort_index()
    assert p.shape==(30,3) and p.notna().all().all()
    d=p.Ours_rollout-p[base]
    q=g.pivot(index='seed',columns='method',values='joint_completion').sort_index().astype(bool)
    won=int((q.Ours_rollout&~q[base]).sum());lost=int((~q.Ours_rollout&q[base]).sum())
    pairs.append(dict(scenario=scene,factor=factor,baseline=base,n=30,
      C_ours=p.Ours_rollout.mean(),C_baseline=p[base].mean(),delta_C=d.mean(),
      ci_low=bootstrap(d,981+si*100+fi*2+bi)[0],ci_high=bootstrap(d,981+si*100+fi*2+bi)[1],
      p_raw=signflip(d,827+si*100+fi*2+bi),
      joint_ours=int(q.Ours_rollout.sum()),joint_baseline=int(q[base].sum()),
      joint_won=won,joint_lost=lost,mcnemar_p=binomtest(won,won+lost,.5).pvalue if won+lost else 1.))
 paired=pd.DataFrame(pairs);paired['p_holm']=holm(paired.p_raw)
 paired.to_csv(OUT/'paired_comparisons.csv',index=False,encoding='utf-8-sig')
 degradation=[]
 for (scene,method),g in main_df.groupby(['scenario','method']):
  p=g.pivot(index='seed',columns='factor',values='collision_rate').sort_index()
  for f in FACTORS:
   if f=='exact':continue
   d=p[f]-p.exact;ci=bootstrap(d)
   degradation.append(dict(scenario=scene,method=method,factor=f,delta_C_vs_exact=d.mean(),ci_low=ci[0],ci_high=ci[1]))
 pd.DataFrame(degradation).to_csv(OUT/'degradation.csv',index=False,encoding='utf-8-sig')
 decomp=[]
 for (scene,method),g in df[df.factor=='exact'].groupby(['scenario','method']):
  for a,b in [('archive','strict'),('strict','safe')]:
   p=g.pivot(index='seed',columns='version',values='collision_rate').sort_index();d=p[b]-p[a]
   decomp.append(dict(scenario=scene,method=method,transition=a+' -> '+b,delta_C=d.mean(),ci_low=bootstrap(d)[0],ci_high=bootstrap(d)[1]))
 pd.DataFrame(decomp).to_csv(OUT/'decomposition.csv',index=False,encoding='utf-8-sig')
 # Cluster the two scenes by seed in pooled confidence intervals.
 pooled=[]
 for (version,factor,method),g in df.groupby(['version','factor','method']):
  byseed=g.groupby('seed').collision_rate.mean()
  pooled.append(dict(version=version,factor=factor,method=method,n_runs=len(g),n_seed_blocks=30,
    C=g.collision_rate.mean(),C_ci=bootstrap(byseed),joint=int(g.joint_completion.sum()),
    arrival=int(g.arrival.sum()),rs=int(g.any_contact_rs.sum()),rho=g.rho_avg.mean()))
 dump(OUT/'statistics.json',dict(summary=records,pooled=pooled,paired=paired.to_dict('records'),
  audit=dict(total_runs=len(df),main_runs=len(main_df),decomposition_extra_runs=360,errors=0,
             strict_truth_fallbacks=int(df[df.version!='archive'].sense_animal_fallback_ticks.sum()),
             source_sha256=source_hash(),safe_rs_contact_runs=int(main_df.any_contact_rs.sum()),
             safe_ro_contact_runs=int(main_df.any_contact_ro.sum()),
             safe_rr_contact_runs=int(main_df.any_contact_rr.sum()),
             safe_robot_violations=int(main_df[['violation_R1','violation_R2','violation_R3','violation_R4_src','violation_lat_src']].sum().sum()),
             total_wall_compute_s=float(df.wall_s.sum()))))
 plt.rcParams.update({'font.family':'DejaVu Sans','font.size':13,'axes.spines.top':False,'axes.spines.right':False,
                      'axes.edgecolor':'#bcc7ca','axes.labelcolor':'#263c48','text.color':'#263c48',
                      'figure.facecolor':'white','savefig.facecolor':'white'})
 for scene in ['V2S2R2','V2S3']:
  fig,axs=plt.subplots(3,1,figsize=(11.2,9.0),sharex=True,gridspec_kw={'height_ratios':[1.5,1,1]})
  x=np.arange(11);s=summary[(summary.version=='safe')&(summary.scenario==scene)].set_index(['factor','method'])
  for mi,m in enumerate(METHODS):
   gg=[s.loc[f,m] for f in FACTORS]
   axs[0].plot(x+(mi-1)*.16,[a.C for a in gg],'o',color=COLORS[mi],label=m.replace('_rollout',''),lw=1.5,ms=4)
   axs[1].plot(x+(mi-1)*.16,[a.joint/30 for a in gg],'o',color=COLORS[mi],lw=1.5,ms=4)
   axs[2].plot(x+(mi-1)*.16,[a.rs_contact_runs/30 for a in gg],'o',color=COLORS[mi],lw=1.5,ms=4)
  axs[0].set_ylabel('Animal-obstacle contact C');axs[1].set_ylabel('Joint completion');axs[2].set_ylabel('Robot-animal\noverlap rate')
  axs[0].legend(ncol=3,frameon=False,loc='upper left',bbox_to_anchor=(0,1.21))
  for ax in axs:ax.grid(axis='y',color='#e1e8e9',lw=.7);ax.set_ylim(-.025,1.025 if ax!=axs[0] else None)
  axs[2].set_xticks(x,LABELS,rotation=38,ha='right')
  fig.suptitle(f'{scene} | shared observation-only safety layer | 30 paired seeds',fontsize=14,x=.07,ha='left')
  fig.tight_layout(rect=[0,0,1,.95]);fig.savefig(FIG/f'{scene}.png',dpi=180);fig.savefig(FIG/f'{scene}.pdf');plt.close(fig)
 fig,axs=plt.subplots(1,2,figsize=(11.2,4.8))
 for ax,scene in zip(axs,['V2S2R2','V2S3']):
  g=summary[(summary.factor=='exact')&(summary.scenario==scene)].set_index(['version','method'])
  for mi,m in enumerate(METHODS):
   y=[g.loc[v,m].rs_clearance_mean for v in ['archive','strict','safe']]
   ax.plot(range(3),y,'-o',color=COLORS[mi],label=m.replace('_rollout',''))
  ax.axhline(0,color='#aa5555',lw=1);ax.set_title(scene);ax.set_xticks(range(3),['Archive','Strict input','Strict + safety']);ax.grid(axis='y',color='#e1e8e9')
 axs[0].set_ylabel('Mean minimum robot-animal\nclearance per run (m)');axs[1].legend(frameon=False)
 fig.tight_layout();fig.savefig(FIG/'safety_decomposition.png',dpi=180);fig.savefig(FIG/'safety_decomposition.pdf');plt.close(fig)
 print(json.dumps(clean({'audit':json.loads((OUT/'statistics.json').read_text())['audit'],'pooled':pooled}),ensure_ascii=False,indent=2))
if __name__=='__main__':main()
