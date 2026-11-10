"""Static scientific measurement figure; no inference from concurrent timing."""
from pathlib import Path
import csv
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'analysis/online_measurement'
with (OUT/'detection_summary.csv').open(encoding='utf-8-sig') as fp:
    rows=list(csv.DictReader(fp))
factors=['exact','animal_pos_0p5','delay_0p5','loss_0p10','blackout_1p0']
labels=['Exact','Animal position\n0.5 m/axis','Delay\n0.5 s','IID link loss\n10%','Shared outage\n1 s']
data={r['factor']:r for r in rows if r['method']=='Ours_rollout' and r['channel']=='animal'}
fig,ax=plt.subplots(1,2,figsize=(13.4,4.7),gridspec_kw={'width_ratios':[1.1,1]})
fig.patch.set_facecolor('white')
for a in ax:
    a.spines[['top','right']].set_visible(False)
    a.grid(axis='y',color='#e0e6e8',lw=.6,zorder=0)
x=np.arange(len(factors));width=.24
for j,(field,label,color) in enumerate([
    ('source_link_missing_fraction','Source link sampling','#8b9ca6'),
    ('delivery_missing_fraction','Fresh sample delivery after fusion','#bb6548'),
    ('current_track_unavailable_fraction','Current track unavailable','#218278')]):
    values=[100*float(data[f][field]) for f in factors]
    bars=ax[0].bar(x+(j-1)*width,values,width,color=color,label=label,zorder=3)
    for b,v in zip(bars,values):
        if v>.05:ax[0].text(b.get_x()+b.get_width()/2,v+.13,f'{v:.1f}',ha='center',va='bottom',fontsize=8)
ax[0].set_xticks(x,labels,fontsize=9)
ax[0].set_ylim(0,13.2);ax[0].set_ylabel('Missing / unavailable target-samples (%)')
ax[0].set_title('Animal-channel information loss (Ours)',loc='left',fontsize=12,pad=12)
ax[0].legend(frameon=False,fontsize=8,loc='upper left')

plan=json.loads((OUT/'protocol.json').read_text())
colors={'exact':'#218278','animal_pos_0p5':'#bb6548','delay_0p5':'#3d6795','blackout_1p0':'#a28a44'}
names={'exact':'Exact','animal_pos_0p5':'Animal position 0.5 m/axis','delay_0p5':'Delay 0.5 s','blackout_1p0':'Shared outage 1 s'}
maximum=0
for factor in colors:
    values=[]
    for s,m,f,seed in plan['timing_cells']:
        if m!='Ours_rollout' or f!=factor:continue
        stem='__'.join((s,m,f,str(seed)))
        with np.load(OUT/'timing_traces'/(stem+'.npz'),allow_pickle=False) as z:
            work=sum(z[p+'_ns'] for p in ('sense_update','sense_publish','controller','fleet_total'))/1e6
            step=z['step']
            for c in np.unique(step//2):
                mask=(step//2==c)
                if c>=10 and np.count_nonzero(mask)==2:values.append(float(work[mask].sum()))
    values=np.sort(values);maximum=max(maximum,float(values.max()))
    ax[1].plot(values,np.arange(1,len(values)+1)/len(values),color=colors[factor],lw=1.4,label=names[factor])
ax[1].axvline(100,color='#884b51',lw=1,ls='--',label='100 ms software budget')
ax[1].set_xscale('log');ax[1].set_xlim(.5,max(130,maximum*1.15));ax[1].set_ylim(0,1.015)
ax[1].set_xlabel('Online software work per 0.1 s cycle (ms; log scale)')
ax[1].set_ylabel('Empirical cumulative fraction of cycles')
ax[1].set_title('Independent serial timing (Ours)',loc='left',fontsize=12,pad=12)
ax[1].legend(frameon=False,fontsize=8,loc='lower right')
fig.text(.055,.018,'Event sample: 2 scenes x 2 seeds per condition. Timing: 2 scenes x seed 3000. Frozen controller; measurement sample only.',fontsize=8,color='#536269')
fig.subplots_adjust(left=.055,right=.985,bottom=.2,top=.89,wspace=.28)
fig.savefig(OUT/'measurement_summary.png',dpi=180,facecolor='white')
plt.close(fig)
