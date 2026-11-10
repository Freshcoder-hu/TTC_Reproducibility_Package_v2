"""Analyze the pre-specified read-only measurement sample, not the primary N."""
from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path
from collections import Counter

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'analysis/online_measurement'
PHASES=('sense_update','sense_publish','controller','guard','feasibility','rs_safety','projection','fleet_total')


def write_csv(path, rows):
    if not rows:return
    with path.open('w',newline='',encoding='utf-8-sig') as fp:
        w=csv.DictWriter(fp,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def finite(value):
    return value is not None and math.isfinite(value)


def json_scalar(value):
    if isinstance(value,np.generic):return value.item()
    raise TypeError(f'Unsupported JSON value: {type(value).__name__}')


def first_index(mask):
    indices=np.flatnonzero(mask)
    return int(indices[0]) if len(indices) else None


def quantiles(values):
    a=np.asarray(values,dtype=float)
    if not len(a):return dict(n=0,median=None,p95=None,p99=None,maximum=None,mean=None)
    return dict(n=len(a),median=float(np.median(a)),p95=float(np.quantile(a,.95)),
                p99=float(np.quantile(a,.99)),maximum=float(a.max()),mean=float(a.mean()))


def pc(value):
    return '未定义' if value is None else f'{100*value:.3f}%'


def main():
    plan=json.loads((OUT/'protocol.json').read_text())
    status=json.loads((OUT/'execution_status.json').read_text())
    assert status['status']=='complete' and status['measurement_replays']==132
    assert status['serial_timing_replays']==24
    assert status['formal_data_sha256']=='d8523bd9e2349cc68576454429ffd3638c4b5f1f918649b2fcee5ef8d062496d'
    runs=[json.loads(x) for x in (OUT/'runs.jsonl').read_text().splitlines()]
    assert len(runs)==132 and len({r['stem'] for r in runs})==132
    assert all(not r.get('error') and r['formal_scalar_parity'] for r in runs)
    assert hashlib.sha256((ROOT/'analysis/measure_online_evidence.py').read_bytes()).hexdigest()==plan['observer_sha256']
    detection=[];events=[];latency=[];components=[];worst_candidates=[]
    groups={}
    for r in runs:
        with np.load(OUT/'traces'/(r['stem']+'.npz'),allow_pickle=False) as f:
            d={k:f[k] for k in f.files}
        t=d['t'];step=d['step'];ctrl=d['ctrl_tick']
        identity={k:r[k] for k in ('scenario','method','factor','seed')}
        if r['result'].get('n_preexisting_contact_so',0)==0:
            assert bool(r['first_so_contact'])==r['result']['any_contact_so']
        for channel,code in (('animal','a'),('obstacle','o')):
            expected=int(d[code+'_due'].sum());delivered=int(d[code+'_delivered'].sum())
            assert int(d[code+'_eligible'].sum())==expected+r['censored_'+channel+'_opportunities']
            eligible=d[code+'_current_eligible_bits'];avail=d[code+'_available_bits']
            current_expected=sum(int(x).bit_count() for x in eligible)
            current_available=sum((int(x)&int(y)).bit_count() for x,y in zip(eligible,avail))
            links=int(d[code+'_link_eligible'].sum());generated=int(d[code+'_link_generated'].sum())
            assert 0<=delivered<=expected and 0<=generated<=links
            # Each matured source target-time is in exactly one success/miss bin.
            raw_expected=sum(int(x).bit_count() for x in d[code+'_due_eligible_bits'])
            raw_delivered=sum(int(x).bit_count() for x in d[code+'_due_delivered_bits'])
            assert (raw_expected,raw_delivered)==(expected,delivered)
            detection.append(dict(**identity,channel=channel,eligible_matured_target_samples=expected,
                delivered_matching_source_samples=delivered,missing_source_samples=expected-delivered,
                delivery_missing_fraction=(expected-delivered)/expected if expected else None,
                right_censored_target_samples=r['censored_'+channel+'_opportunities'],
                eligible_robot_target_samples=links,generated_robot_target_samples=generated,
                source_link_missing_fraction=(links-generated)/links if links else None,
                current_eligible_target_ticks=current_expected,available_target_ticks=current_available,
                current_track_unavailable_fraction=(current_expected-current_available)/current_expected if current_expected else None))

        n_obstacles=d['truth_ttc'].shape[1]
        for k in range(n_obstacles):
            ref=first_index(ctrl&(d['truth_ttc'][:,k]<=12.))
            tr=float(t[ref]) if ref is not None else None
            projected=tr+float(d['truth_ttc'][ref,k]) if ref is not None else None
            lo=max(0.,tr-12.) if tr is not None else 0.
            info=first_index((t>=lo-1e-8)&(d['observed_ttc'][:,k]<=12.))
            policy=first_index((t>=lo-1e-8)&ctrl&(d['policy_target']==k))
            delivery=first_index(np.isfinite(d['obstacle_source_stamps'][:,k]))
            eligible=first_index((d['o_current_eligible_bits']&np.uint64(1<<k))!=0)
            ti=float(t[info]) if info is not None else None
            tp=float(t[policy]) if policy is not None else None
            contact=r['first_so_contact'].get(str(k))
            def warning_status(when,implemented=True):
                if not implemented:return 'not_implemented'
                if tr is None:return 'no_reference_threat'
                if when is None:
                    return 'right_censored' if float(t[-1])<projected-1e-8 else 'no_warning'
                return 'on_time' if when<=projected+1e-8 else 'late'
            def contact_status(when,implemented=True):
                if not implemented:return 'not_implemented'
                if contact is None:return 'no_contact'
                if when is None:return 'no_warning'
                return 'before_contact' if when<contact-1e-8 else 'late_or_at_contact'
            info_status=warning_status(ti)
            # An event window ending at the frozen reference impact defines
            # on-time status; later warnings remain explicit late observations.
            events.append(dict(**identity,obstacle_id=k,reference_threat=ref is not None,
                reference_t=tr,reference_projected_contact_t=projected,
                warning_followup_end_t=float(t[-1]),
                reference_followup_complete=bool(projected is not None and float(t[-1])>=projected-1e-8),
                first_range_eligible_t=float(t[eligible]) if eligible is not None else None,
                first_obstacle_publication_t=float(t[delivery]) if delivery is not None else None,
                first_information_warning_t=ti,
                information_warning_source_stamp=float(d['obstacle_source_stamps'][info,k]) if info is not None else None,
                information_warning_age_s=ti-float(d['obstacle_source_stamps'][info,k]) if info is not None else None,
                information_warning_status=info_status,
                information_projected_lead_s=projected-ti if projected is not None and ti is not None else None,
                information_onset_lag_s=ti-tr if tr is not None and ti is not None else None,
                policy_implemented=r['method']!='B1',first_policy_response_t=tp,
                policy_response_status=warning_status(tp,r['method']!='B1'),
                policy_projected_lead_s=projected-tp if projected is not None and tp is not None else None,
                first_actual_contact_t=contact,
                information_actual_contact_lead_s=contact-ti if contact is not None and ti is not None else None,
                policy_actual_contact_lead_s=contact-tp if contact is not None and tp is not None else None,
                information_actual_contact_status=contact_status(ti),
                policy_actual_contact_status=contact_status(tp,r['method']!='B1')))

    # Concurrent event acquisition wall times are NEVER used as latency data.
    timing_runs=[json.loads(x) for x in (OUT/'timing_runs.jsonl').read_text().splitlines()]
    assert len(timing_runs)==24 and len({r['stem'] for r in timing_runs})==24
    event_lookup={r['stem']:r for r in runs}
    for r in timing_runs:
        assert r['measurement_kind']=='serial_timing' and r['formal_scalar_parity']
        for key in ('trajectory_sha256','final_commands_sha256','controller_commands_sha256'):
            assert r[key]==event_lookup[r['stem']][key],(r['stem'],key)
        with np.load(OUT/'timing_traces'/(r['stem']+'.npz'),allow_pickle=False) as f:
            d={k:f[k] for k in f.files}
        identity={k:r[k] for k in ('scenario','method','factor','seed')}
        step=d['step']
        online=sum(d[p+'_ns'] for p in ('sense_update','sense_publish','controller','fleet_total'))/1e6
        online_cpu=sum(d[p+'_cpu_ns'] for p in ('sense_update','sense_publish','controller','fleet_total'))/1e6
        for j in np.argsort(online)[-10:]:
            worst_candidates.append(dict(**identity,step=int(step[j]),t=float(d['t'][j]),
                online_wall_ms=float(online[j]),online_cpu_ms=float(online_cpu[j]),
                over_50ms=bool(online[j]>50.),
                **{p+'_ms':float(d[p+'_ns'][j])/1e6 for p in PHASES}))
        post=step>=20  # first ten 0.1 s controller cycles kept in raw files
        cycle_values=[];cycle_cpu=[];all_cycle_values=[]
        partial_cycles=0
        for cycle in np.unique(step//2):
            mask=(step//2==cycle)
            if np.count_nonzero(mask)!=2:
                partial_cycles+=1;continue
            all_cycle_values.append(float(online[mask].sum()))
            if cycle<10:continue
            cycle_values.append(float(online[mask].sum()))
            cycle_cpu.append(float(online_cpu[mask].sum()))
        q=quantiles(online[post]);qc=quantiles(cycle_values)
        latency.append(dict(**identity,steps=q['n'],step_mean_ms=q['mean'],step_median_ms=q['median'],
            step_p95_ms=q['p95'],step_p99_ms=q['p99'],step_max_ms=q['maximum'],
            step_over_50ms=int(np.sum(online[post]>50.)),
            complete_cycles=qc['n'],cycle_mean_ms=qc['mean'],cycle_median_ms=qc['median'],
            cycle_p95_ms=qc['p95'],cycle_p99_ms=qc['p99'],cycle_max_ms=qc['maximum'],
            cycle_over_100ms=sum(x>100 for x in cycle_values),partial_last_cycles=partial_cycles,
            including_warmup_step_max_ms=float(online.max()),
            including_warmup_step_over_50ms=int(np.sum(online>50.)),
            including_warmup_cycle_max_ms=max(all_cycle_values),
            including_warmup_cycle_over_100ms=sum(x>100 for x in all_cycle_values),
            cycle_cpu_mean_ms=float(np.mean(cycle_cpu))))
        for phase in PHASES:
            # Keep zero-cost no-call ticks: these are per-service-step costs.
            qphase=quantiles(d[phase+'_ns'][post]/1e6)
            components.append(dict(**identity,phase=phase,**{k+'_ms' if k!='n' else k:v for k,v in qphase.items()}))
        key=(r['method'],r['factor'])
        g=groups.setdefault(key,dict(n_runs=0,steps=[],cycles=[],all_steps=[],all_cycles=[],cpu=[],phases={p:[] for p in PHASES}))
        g['n_runs']+=1
        g['steps'].extend(online[post]);g['cycles'].extend(cycle_values);g['cpu'].extend(cycle_cpu)
        g['all_steps'].extend(online);g['all_cycles'].extend(all_cycle_values)
        for phase in PHASES:g['phases'][phase].extend(d[phase+'_ns'][post]/1e6)

    write_csv(OUT/'detection_by_run.csv',detection)
    write_csv(OUT/'warning_events.csv',events)
    write_csv(OUT/'latency_by_run.csv',latency)
    write_csv(OUT/'phase_timing_by_run.csv',components)
    write_csv(OUT/'worst_steps.csv',sorted(worst_candidates,key=lambda x:x['online_wall_ms'],reverse=True)[:20])
    detection_summary=[]
    for method in ('Ours_rollout','B1','B2'):
        for factor in dict.fromkeys(r['factor'] for r in runs):
            for channel in ('animal','obstacle'):
                rs=[x for x in detection if x['method']==method and x['factor']==factor and x['channel']==channel]
                totals={k:sum(x[k] for x in rs) for k in (
                    'eligible_matured_target_samples','delivered_matching_source_samples','missing_source_samples',
                    'right_censored_target_samples','eligible_robot_target_samples','generated_robot_target_samples',
                    'current_eligible_target_ticks','available_target_ticks')}
                n=totals['eligible_matured_target_samples'];lk=totals['eligible_robot_target_samples'];cur=totals['current_eligible_target_ticks']
                detection_summary.append(dict(method=method,factor=factor,channel=channel,runs=len(rs),**totals,
                    delivery_missing_fraction=totals['missing_source_samples']/n if n else None,
                    source_link_missing_fraction=(lk-totals['generated_robot_target_samples'])/lk if lk else None,
                    current_track_unavailable_fraction=(cur-totals['available_target_ticks'])/cur if cur else None))
    warning_summary=[]
    timing_summary=[]
    event_groups=dict.fromkeys((r['method'],r['factor']) for r in runs)
    for method,factor in event_groups:
        es=[e for e in events if e['method']==method and e['factor']==factor]
        ref=[e for e in es if e['reference_threat']]
        ci=Counter(e['information_warning_status'] for e in ref)
        cp=Counter(e['policy_response_status'] for e in ref)
        ct=Counter(e['information_actual_contact_status'] for e in es if e['first_actual_contact_t'] is not None)
        lead=[e['information_projected_lead_s'] for e in ref if finite(e['information_projected_lead_s'])]
        policy_lead=[e['policy_projected_lead_s'] for e in ref if finite(e['policy_projected_lead_s'])]
        warning_summary.append(dict(method=method,factor=factor,potential_encounters=len(es),
            reference_threat_events=len(ref),no_reference_threat=len(es)-len(ref),
            information_on_time=ci['on_time'],information_late=ci['late'],information_no_warning=ci['no_warning'],
            information_right_censored=ci['right_censored'],
            policy_on_time=cp['on_time'],policy_late=cp['late'],policy_no_warning=cp['no_warning'],
            policy_right_censored=cp['right_censored'],
            policy_not_implemented=cp['not_implemented'],
            information_projected_lead_median_s=float(np.median(lead)) if lead else None,
            information_projected_lead_min_s=float(min(lead)) if lead else None,
            policy_projected_lead_median_s=float(np.median(policy_lead)) if policy_lead else None,
            actual_contact_encounters=sum(e['first_actual_contact_t'] is not None for e in es),
            actual_contact_without_reference=sum(e['first_actual_contact_t'] is not None and not e['reference_threat'] for e in es),
            contacts_information_warned_before=ct['before_contact'],contacts_information_late=ct['late_or_at_contact'],
            contacts_information_no_warning=ct['no_warning']))
        g=groups.get((method,factor))
        if g is None:continue
        q=quantiles(g['steps']);qc=quantiles(g['cycles'])
        timing_summary.append(dict(method=method,factor=factor,runs=g['n_runs'],
            step_count=q['n'],step_p50_ms=q['median'],step_p95_ms=q['p95'],step_p99_ms=q['p99'],step_max_ms=q['maximum'],
            step_over_50ms=sum(x>50 for x in g['steps']),
            cycle_count=qc['n'],cycle_p50_ms=qc['median'],cycle_p95_ms=qc['p95'],cycle_p99_ms=qc['p99'],cycle_max_ms=qc['maximum'],
            cycle_over_100ms=sum(x>100 for x in g['cycles']),
            including_warmup_step_max_ms=float(max(g['all_steps'])),
            including_warmup_step_over_50ms=sum(x>50 for x in g['all_steps']),
            including_warmup_cycle_max_ms=float(max(g['all_cycles'])),
            including_warmup_cycle_over_100ms=sum(x>100 for x in g['all_cycles']),
            cycle_cpu_mean_ms=float(np.mean(g['cpu'])),
            **{p+'_step_mean_ms':float(np.mean(g['phases'][p])) for p in PHASES}))
    write_csv(OUT/'detection_summary.csv',detection_summary)
    write_csv(OUT/'warning_summary.csv',warning_summary)
    write_csv(OUT/'timing_summary.csv',timing_summary)
    method_overview=[]
    for method in ('Ours_rollout','B1','B2'):
        es=[e for e in events if e['method']==method]
        ts=[e for e in timing_summary if e['method']==method]
        refs=[e for e in es if e['reference_threat']]
        lead=[e['policy_projected_lead_s'] for e in refs if finite(e['policy_projected_lead_s'])]
        actual=[e['policy_actual_contact_lead_s'] for e in es if finite(e['policy_actual_contact_lead_s'])]
        method_overview.append(dict(method=method,reference_threats=len(refs),
            information_on_time=sum(e['information_warning_status']=='on_time' for e in refs),
            policy_on_time=sum(e['policy_response_status']=='on_time' for e in refs),
            policy_implemented=method!='B1',
            policy_projected_lead_median_s=float(np.median(lead)) if lead else None,
            policy_projected_lead_min_s=float(min(lead)) if lead else None,
            contacts_with_policy_lead=len(actual),
            policy_actual_contact_lead_median_s=float(np.median(actual)) if actual else None,
            policy_actual_contact_lead_min_s=float(min(actual)) if actual else None,
            serial_timing_runs=sum(t['runs'] for t in ts),
            step_over_50ms=sum(t['step_over_50ms'] for t in ts),
            cycle_over_100ms=sum(t['cycle_over_100ms'] for t in ts),
            maximum_cycle_ms=max(t['cycle_max_ms'] for t in ts)))
    write_csv(OUT/'method_overview.csv',method_overview)

    formal=[json.loads(x) for x in (ROOT/'results/formal.jsonl').read_text().splitlines()]
    legacy=[r for r in formal if r['version']=='safe' and not finite(r.get('ttc_lead'))]
    legacy_summary=dict(nonfinite_total=len(legacy),by_method=dict(Counter(r['method'] for r in legacy)),
        original_definition='truth distance threshold onset minus truth TTC threshold onset, only if controller exposes t_pre',
        not_a_delivery_warning_metric=True)
    validation=dict(protocol_and_observer_hash_match=True,expected_unique_runs=132,
        raw_bitmask_counts_match_summary=True,formal_data_unchanged=True,
        formal_scalar_parity_runs=132,full_trajectory_and_command_parity_runs=status['full_trajectory_and_command_parity_runs'],
        serial_timing_scalar_parity_runs=24,serial_event_full_trajectory_command_parity_runs=24,
        concurrent_event_timings_excluded_from_latency=True,
        all_potential_encounters_retained=len(events),reference_threat_events=sum(e['reference_threat'] for e in events),
        censored_sources_not_counted_as_misses=True,
        max_censor_count=sum(r['censored_source_samples'] for r in runs))
    (OUT/'analysis_validation.json').write_text(json.dumps(validation,indent=2))
    (OUT/'legacy_warning_audit.json').write_text(json.dumps(legacy_summary,indent=2))

    ours_d={r['factor']:r for r in detection_summary if r['method']=='Ours_rollout' and r['channel']=='animal'}
    ours_w={r['factor']:r for r in warning_summary if r['method']=='Ours_rollout'}
    ours_t={r['factor']:r for r in timing_summary if r['method']=='Ours_rollout'}
    factors=('exact','animal_pos_0p5','delay_0p5','loss_0p10','blackout_1p0')
    lines=[]
    for f in factors:
        dr,wr,tm=ours_d[f],ours_w[f],ours_t.get(f)
        timing=(f"100 ms 周期 p99 {tm['cycle_p99_ms']:.3f} ms，观测最大 {tm['cycle_max_ms']:.3f} ms，超期 {tm['cycle_over_100ms']}/{tm['cycle_count']}。" if tm else '该条件未纳入独立串行计时面板。')
        lines.append(f"{f}: 样本交付缺失 {pc(dr['delivery_missing_fraction'])}；当前轨迹不可用 {pc(dr['current_track_unavailable_fraction'])}；"
            f"参考威胁 {wr['reference_threat_events']}，信息按时/晚到/无预警 {wr['information_on_time']}/{wr['information_late']}/{wr['information_no_warning']}；"
            +timing)
    allsteps=[x for g in groups.values() for x in g['steps']]
    allcycles=[x for g in groups.values() for x in g['cycles']]
    startup_steps=[x for g in groups.values() for x in g['all_steps']]
    startup_cycles=[x for g in groups.values() for x in g['all_cycles']]
    result=dict(status='complete',sample_runs=132,serial_timing_runs=24,performance_N_unchanged=2340,
        method_overview=method_overview,
        seeds=[3000,3029],full_trajectory_and_command_parity_runs=status['full_trajectory_and_command_parity_runs'],
        legacy_warning_audit=legacy_summary,validation=validation,
        all_steps=quantiles(allsteps),all_cycles=quantiles(allcycles),
        steps_over_50ms=int(sum(x>50 for x in allsteps)),cycles_over_100ms=int(sum(x>100 for x in allcycles)),
        including_warmup_steps_over_50ms=int(sum(x>50 for x in startup_steps)),
        including_warmup_cycles_over_100ms=int(sum(x>100 for x in startup_cycles)),
        including_warmup_max_step_ms=float(max(startup_steps)),
        including_warmup_max_cycle_ms=float(max(startup_cycles)),
        warning_status=dict(Counter(e['information_warning_status'] for e in events)),
        caveat='measurement sample only; information availability is separate from actual policy response; host software timing is not hardware WCET')
    our_overview=method_overview[0]
    followup_path=OUT/'overhead_checks/summary.json'
    followup=json.loads(followup_path.read_text()) if followup_path.exists() else None
    followup_text=''
    if followup:
        result['targeted_timing_checks']=followup
        followup_text='\n针对峰值的低分配复核（事后补充，未并入上述 24 次面板）：\n'
        for case in followup['results']:
            followup_text+=(f"{case['method']}: 不保留完整物理轨迹和事件诊断数组，仍启用 GC；"
                f"周期观测最大 {case['cycle_max_ms']:.3f} ms，超过 100 ms {case['cycle_over_100ms']} 次；"
                f"原生 RS 核最大 {case['native_rs_max_ms']:.3f} ms，原生 Rollout 核最大 {case['native_rollout_max_ms']:.3f} ms；"
                f"观测 GC 最大暂停 {case['max_observed_gc_pause_ms']:.3f} ms。\n")
        followup_text+='记录器的内存保留和 GC 行为可能改变峰值；原面板仍全部保留。不能把 RS 包装函数的全部耗时都归给原生计算核，也不能据两个定点复核宣称最坏时延已获保证。\n'
    (OUT/'summary.json').write_text(json.dumps(result,indent=2,default=json_scalar))
    text=f'''TTC：失检、实际预警与在线计算耗时的测量补充
日期：2026-10-29

这是一组只增加记录的独立测量回放，未修改已冻结的控制策略、执行层、
感知模型或正式性能结果。原性能样本仍为 1,980 + 360 = 2,340，不能加上本次回放。

范围与不干扰验证
2 场景 × 3 控制器 × 11 感知条件 × 2 预选种子（3000、3029）= 132 次测量。
两种子按原评价序号首尾选取，没有依据表现选取。该样本不能替代 30 种子性能推断。
全部 132 次与原正式结果的共同确定性标量逐项一致；另有 9 个预定案例，
完整积分轨迹、控制器原始输出和最终执行指令序列均与轻量观测对照逐字节一致。
事件记录可以并行采集，其受争用影响的计时全部排除在时延分析之外。
独立串行计时面板有 24 次：2 场景 × 3 控制器 × 4 条件（exact、动物误差 0.5 m、
延迟 0.5 s、共享中断 1 s）× 种子 3000。所有事件采集进程结束后才开始计时。
该面板单进程串行、BLAS 单线程并固定一个逻辑 CPU；完整环境见 environment.json。
24 次串行计时回放与对应事件回放的完整轨迹、原始指令和最终指令指纹也均一致。
调度预检的 4 次串行回放另存，未并入本次样本；修改的只有采集调度和计时面板，
没有改变指标定义或控制策略，相关说明见 online_measurement_serial_precheck/。
生产源码和原生库指纹仍为 {status['frozen_source_sha256']}。

一、失检：分别报告来源采样缺失、实际交付缺失与当前轨迹不可用
范围条件由真实状态离线判定，只进入诊断，不进入控制器或安全层。
主分母是至少一台机器人应能探测的“唯一目标—源采样时刻”，不是空观测步数。
到 nominal source_time+delay 的实际发布帧，只有目标身份和源时间戳均匹配
才记作该样本已有效交付；此前旧缓存存在并不代表这条新样本已交付。
运行结束时尚未到名义交付时间的源样本保留为右删失，不计作漏交付。
另报告每机器人—目标采样在进入缓存前的缺失，以及当前可探测目标是否有
未过期的已发布轨迹。三者的分母及意义不同，不能混用。
本仿真仍没有视觉识别、假阳性目标或遮挡渲染，因此这里是端到端采样交付指标，
不能称为真实视觉检测器的漏检率。
目标时刻存在时间与机器人之间的相关性，计数只作描述，不当作大量独立样本推断。

二、预警：信息已具备预警条件，与控制器实际响应分开
每个障碍物释放作为一个潜在遭遇，逐个保留。参考事件是控制时刻首次真值
常速度实体 TTC≤12 s；参考碰撞时刻固定为此时刻加该 TTC。这是局部外推参照，
不是已经发生的物理碰撞，也不假设之后动物保持恒速。
以障碍物身份匹配，在 t_ref-12 s 起的事件窗口中记录：
1. 发布观测的点估计首次能给出 TTC≤12 s 的时刻，称“信息可预警时刻”；
2. Ours/B2 实际发出且归因于该障碍物的 PRE/DIVERT 请求时刻。
前者对所有方法使用共同的只读诊断，并不声称 B1/B2 内部实现了 TTC 报警。
B1 的高层障碍预警能力记作“不适用/未实现”，不能算成 100% 漏警。
预警迟于固定参考碰撞时刻明确记为晚到；始终未预警保留为无预警。
若运行在参考时限前结束且此前尚未预警，则单列为右删失，不冒充已经证实的漏警。
没有参考风险的潜在遭遇也保留，不能加入漏警分母。
发生实际动物—障碍物接触时，另以连续扫掠检测的首次接触时间计算实际提前量；
没有接触的成功避让不填补虚构的碰撞时间。高层请求不等同于过滤后的实际动作。

Ours 的 {our_overview['reference_threats']} 个参考威胁中，信息可预警及实际高层响应均有
{our_overview['policy_on_time']} 个先于所定义的参考碰撞时限；实际响应相对该参照的提前量
中位数为 {our_overview['policy_projected_lead_median_s']:.3f} s，最小 {our_overview['policy_projected_lead_min_s']:.3f} s。
在本次 {our_overview['contacts_with_policy_lead']} 个有实际接触且能匹配高层响应的遭遇中，
实际接触前的响应提前量中位数为 {our_overview['policy_actual_contact_lead_median_s']:.3f} s，
最小 {our_overview['policy_actual_contact_lead_min_s']:.3f} s。
这说明预警存在仍不保证能完成避险，不能把“按时预警”解释成“没有接触”。

旧 ttc_lead 的 1,328 条非有限记录中，B1 为 660 条、B2 为 660 条、Ours 为 8 条。
旧定义依赖控制器的 t_pre，B1/B2 没有该属性，因此其中 1,320 条是定义不适用，
不是证实了 1,320 次漏警。Ours 其余 8 条仍须按事件记录判断，而不能由 NaN 判定。

三、计算耗时：覆盖在线软件路径的所有主要部分
记录感知采样/更新、感知发布/融合、控制器、机器人守卫、可行性过滤、RS 安全层、
运动投影，以及包含守卫/过滤/投影/位置积分的整个 fleet.step。
在线工作耗时 = 感知更新 + 发布 + 控制器 + 整个 fleet.step；
内部各阶段只用于归因，不再加一次，避免重复计算。
原始逐步文件同时记录墙钟与进程 CPU 时间。首 10 个控制周期标为预热，
统计同时给出 50 ms 积分步预算与完整两步 100 ms 控制周期预算。
预热期间的最大耗时和超期计数另列，未从原始证据中删除。
这是本机软件在线服务的观测耗时，不含相机采集、真实网络服务或动物仿真/
碰撞指标等离线诊断，也不是硬件最坏执行时间的证明。fleet 外层计时包含少量
只读记录器开销，未人为扣除。其余分析在计时区域之外执行。
设定的通信延迟按仿真时间单列；计算预算达标不表示感知至执行的信息总年龄
也小于 100 ms，不能用短计算时间代替端到端通信与数据时效的论证。
仿真仍按固定 0.05 s 积分步推进，测得的计算耗时没有反注入控制延迟或动物运动。
因此正式性能结果不包含计算超期导致的额外闭环退化，不能据此直接预测硬件部署结果。

Ours 两场景合并的描述性测量（事件每条件 4 次回放；串行耗时每条件 2 次）：
'''+ '\n'.join(lines)+f'''

串行计时面板共有 {len(allsteps)} 个预热后积分步、{len(allcycles)} 个完整控制周期。
观测到积分步超过 50 ms：{result['steps_over_50ms']}；控制周期超过 100 ms：{result['cycles_over_100ms']}。
含预热时相应超期为 {result['including_warmup_steps_over_50ms']} 和 {result['including_warmup_cycles_over_100ms']}，
对应单步/完整周期最大值为 {result['including_warmup_max_step_ms']:.3f}/{result['including_warmup_max_cycle_ms']:.3f} ms。
按方法区分：Ours 有 {our_overview['step_over_50ms']} 个 50 ms 单步超期，
100 ms 周期超期 {our_overview['cycle_over_100ms']} 次，周期观测最大 {our_overview['maximum_cycle_ms']:.3f} ms。
整个面板的 208 ms 周期峰值属于 B2 的共享中断条件，不能归到 Ours 名下。
所有超期均保留；无论是否观察到超期，都不能据此宣称所有硬件或未来状态均实时。
{followup_text}

证据入口
- analysis/online_measurement/protocol.json：测量前冻结的范围、定义和排程。
- detection_summary.csv / detection_by_run.csv：分母、有效交付、缺失和右删失。
- warning_events.csv：每个潜在遭遇的参考事件、交付、信息预警、实际响应与接触。
- warning_summary.csv：按方法/感知条件分组的事件状态。
- timing_summary.csv / phase_timing_by_run.csv：逐阶段归因与时限超期。
- traces/*.npz：逐积分步原始掩码、源时间戳、风险和计时，allow_pickle=False 可读取。
- timing_traces/*.npz：独立串行计时面板原始记录；时延表只使用这些文件。
- parity_references.json / runs.jsonl：轨迹/指令指纹及与原结果的逐项核对。

这三项测量定义和本次样本的原始证据已经闭环；对其余评价种子及真实传感硬件
仍未外推。机器人—动物接触未清零，动物定位误差与保守安全裕量仍存在控制代价，
本次测量不改变此前的安全局限和模型失配解释。
'''
    (ROOT/'Online_Evidence_Measurement_ZH.txt').write_text(text,encoding='utf-8')
    print(json.dumps(result,indent=2,default=json_scalar))


if __name__=='__main__':main()
