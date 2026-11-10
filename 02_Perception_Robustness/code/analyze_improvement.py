"""Paired seed-level summaries. No deletion of adverse seeds or infinities."""
import argparse
import json
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.stats import wilcoxon,binomtest

METRICS=['collision_rate','min_dist','rho_avg','max_dev','completion','success']

def read_rows(folder):
    folder=Path(folder)
    active=folder.parent/'ACTIVE_RESULTS.json'
    if active.exists():
        mapping=json.loads(active.read_text())
        folder=folder.parent/mapping.get(folder.name,folder.name)
    return pd.DataFrame([json.loads(p.read_text()) for p in Path(folder).glob('*__*__*.json')])

def paired_difference(a,b,metric,rng,repetitions=30000):
    z=a[['seed',metric]].merge(b[['seed',metric]],on='seed',suffixes=('_new','_ref'))
    delta=(z[metric+'_new'].astype(float)-z[metric+'_ref'].astype(float)).to_numpy()
    delta=delta[np.isfinite(delta)]
    means=delta[rng.integers(0,len(delta),size=(repetitions,len(delta)))].mean(axis=1)
    ci=np.quantile(means,[.025,.975])
    if metric in ('completion','success'):
        discordant=int(np.sum(delta!=0))
        p=1. if discordant==0 else float(binomtest(int(np.sum(delta<0)),discordant,.5).pvalue)
        method='exact_McNemar'
    else:
        p=1. if np.all(delta==0) else float(wilcoxon(delta,alternative='two-sided',zero_method='wilcox').pvalue)
        method='Wilcoxon_two_sided'
    sign=1 if metric in ('collision_rate','max_dev') else -1
    return dict(n=len(delta),delta=float(delta.mean()),ci_low=float(ci[0]),ci_high=float(ci[1]),
                wins=int(np.sum(sign*delta<0)),ties=int(np.sum(delta==0)),
                losses=int(np.sum(sign*delta>0)),p_raw=p,p_method=method)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--root',default='.')
    ap.add_argument('--out',default='analysis_improvement')
    args=ap.parse_args();root=Path(args.root);out=root/args.out;out.mkdir(exist_ok=True)
    ref=read_rows(root/'results_validation_reference')
    new=read_rows(root/'results_validation_rollout')
    data=pd.concat([ref,new],ignore_index=True)
    data.to_csv(out/'s3_validation_detail.csv',index=False,encoding='utf-8-sig')
    data.groupby('variant')[METRICS].agg(['count','mean','std']).to_csv(out/'s3_summary.csv',encoding='utf-8-sig')
    rows=[];rng=np.random.default_rng(78521)
    for variant in sorted(ref.variant.unique()):
        for metric in METRICS:
            a=new[new.variant=='rollout'];b=ref[ref.variant==variant]
            if len(b)!=30 or len(a)!=30:
                continue
            r=paired_difference(a,b,metric,rng)
            rows.append(dict(reference=variant,metric=metric,**r))
    comparisons=pd.DataFrame(rows)
    if len(comparisons):
        order=np.argsort(comparisons.p_raw.to_numpy())
        corrected=np.maximum.accumulate((len(order)-np.arange(len(order)))*comparisons.p_raw.to_numpy()[order])
        comparisons['p_holm']=np.nan
        comparisons.loc[comparisons.index[order],'p_holm']=np.minimum(1,corrected)
        comparisons.to_csv(out/'s3_paired.csv',index=False,encoding='utf-8-sig')
    gen=read_rows(root/'results_generalization')
    if len(gen):
        gen.to_csv(out/'generalization_detail.csv',index=False,encoding='utf-8-sig')
        gen.groupby(['scenario','variant'])[METRICS].agg(['count','mean']).to_csv(out/'generalization_summary.csv',encoding='utf-8-sig')
    fresh=read_rows(root/'results_fresh_confirmation')
    if len(fresh):
        fresh.to_csv(out/'fresh_confirmation_detail.csv',index=False,encoding='utf-8-sig')
        fresh.groupby('variant')[METRICS].agg(['count','mean']).to_csv(out/'fresh_confirmation_summary.csv',encoding='utf-8-sig')
    print(data.groupby('variant')[METRICS].mean().to_string())
    print(comparisons[comparisons.reference.isin(['B2','Ours'])].to_string(index=False) if len(comparisons) else '')

if __name__=='__main__':main()
