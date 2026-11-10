from pathlib import Path
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.size':13})
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'analysis/failure_replay'
frames={f:pd.read_csv(OUT/(f+'.csv')) for f in ['exact','blackout_1p0']}
f=frames['blackout_1p0'];tc=float(f.loc[f.rs_min_swept.idxmin(),'t'])
fig,ax=plt.subplots(2,1,figsize=(10.5,6.2))
for a in ax:
 for name,c,label in [('exact','#137d87','Exact'),('blackout_1p0','#b75f39','1 s shared animal outage')]:
  z=frames[name];a.plot(z.t,z.rs_min_swept,color=c,label=label,lw=1)
 a.axhline(0,color='#9a4949',lw=1);a.grid(axis='y',color='#e0e6e8');a.set_ylabel('Minimum robot-animal\nclearance (m)')
 for begin,end in zip(f.t[(f.n_animals==0)&(f.n_animals.shift(fill_value=36)>0)],f.t[(f.n_animals>0)&(f.n_animals.shift(fill_value=36)==0)]):
  a.axvspan(begin,end,color='#b75f39',alpha=.15)
ax[0].set_title('V2S3 / Ours / seed 3009: dense replay, no controller changes')
ax[0].legend(frameon=False);ax[1].set_xlim(tc-4,tc+4);ax[1].set_ylim(min(f.rs_min_swept.min()-.05,-.1),1.)
ax[1].set_xlabel('Simulation time (s); shaded = no animal observations')
fig.tight_layout();fig.savefig(OUT/'replay.png',dpi=180);plt.close(fig)
