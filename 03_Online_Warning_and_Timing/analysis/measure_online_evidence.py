"""Read-only measurement replay of the frozen Safe pipeline.

No production source is edited. All diagnostics run after the wrapped call has
returned; no extra RNG draws are made. Detailed observations and timings are
separate from the 2340-run performance dataset. Run serially for latency data.
"""
from __future__ import annotations

import os
for _key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_key] = '1'
import argparse
from collections import deque
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import json
import platform
from pathlib import Path
import sys
import time
import traceback
from dataclasses import replace
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'code'))
from run_robustness import config, source_hash, FACTORS, METHODS, SCENES
from v2.runner_s4 import run_cell, ALGS
from v2.scenarios_v2 import resolve_scene
from v2.controller_v2 import make_controller, unpack
from v2.robust_sensing import RobustSensorium
from v2.feasibility_v2 import FeasibilityLayer
from v2.rs_safety import RSSafety
from v2.fleet_v2 import RobotFleetV2
from v2.contacts_v2 import ContactMonitor
from v2 import fleet_v2
from robots import threat_obstacle

OUT = ROOT/'analysis/online_measurement'
PHASES = ('sense_update', 'sense_publish', 'controller', 'guard',
          'feasibility', 'rs_safety', 'projection', 'fleet_total')
SEEDS = (3000, 3029)
EPS = 1e-8


def digest_values(h, values):
    for value in values:
        a = np.ascontiguousarray(value)
        h.update(a.dtype.str.encode()); h.update(str(a.shape).encode()); h.update(a.tobytes())


def digest_trace(trace):
    h = hashlib.sha256()
    for row in trace:
        digest_values(h, row)
    return h.hexdigest()


def min_cv_ttc(sp, sv, op, ov, radii):
    """Physical disc first entry, including existing overlap; offline only."""
    n, k = len(sp), len(op)
    result = np.full(k, np.inf)
    if not n or not k:
        return result
    r = np.asarray(sp)[:, None, :] - np.asarray(op)[None, :, :]
    v = np.asarray(sv)[:, None, :] - np.asarray(ov)[None, :, :]
    aa = np.sum(v*v, axis=2)
    bb = 2*np.sum(r*v, axis=2)
    cc = np.sum(r*r, axis=2)-np.asarray(radii)[None, :]**2
    disc = bb*bb-4*aa*cc
    root = (-bb-np.sqrt(np.maximum(disc, 0)))/np.where(aa>1e-24, 2*aa, 1.)
    valid = (aa>1e-24)&(disc>=0)&(root>=0)
    entry = np.where(cc<=0, 0., np.where(valid, root, np.inf))
    return entry.min(axis=0)


def bits(mask):
    return sum(1 << int(i) for i in np.flatnonzero(mask))


def bit_count(value):
    return int(value).bit_count()


def clock_call(fn, *args, **kwargs):
    w0, c0 = time.perf_counter_ns(), time.process_time_ns()
    out = fn(*args, **kwargs)
    c1, w1 = time.process_time_ns(), time.perf_counter_ns()
    return out, w1-w0, c1-c0


class ReadOnlyProbe:
    def __init__(self, scene, method, factor, seed, detailed=True):
        self.scene, self.method, self.factor, self.seed = scene, method, factor, seed
        self.vc = config('safe', factor)
        self.sc = resolve_scene(scene)
        self.k = len(self.sc['obstacles'])
        self.frames = []
        self.current = None
        self.pending = deque()
        self.commands = hashlib.sha256()
        self.controller_commands = hashlib.sha256()
        self.first_contact = {}
        self.detailed = detailed
        self.last_tag = 'init'
        self.original = dict(update=RobustSensorium.update, publish=RobustSensorium.publish,
            fleet=RobotFleetV2.step, guard=fleet_v2.apply_robot_safety_v2,
            feas=FeasibilityLayer.filter, rs=RSSafety.filter,
            project=fleet_v2.robot_step_project, contact=ContactMonitor.update)

    def timed(self, phase, fn, *args, **kwargs):
        out, wall, cpu = clock_call(fn, *args, **kwargs)
        self.current[phase+'_ns'] += wall
        self.current[phase+'_cpu_ns'] += cpu
        return out

    def update(self, sensor, t, step, ctrl_tick, rp, obs, sp, sv):
        f = dict(t=float(t), step=int(step), ctrl_tick=bool(ctrl_tick))
        for phase in PHASES:
            f[phase+'_ns'] = f[phase+'_cpu_ns'] = 0
        f.update(tag=self.last_tag, policy_target=-1, source_t=np.nan,
                 a_eligible=0, a_generated=0, o_eligible=0, o_generated=0,
                 a_link_eligible=0, a_link_generated=0, o_link_eligible=0, o_link_generated=0,
                 a_due=0, a_delivered=0, o_due=0, o_delivered=0,
                 a_due_eligible_bits=0, a_due_delivered_bits=0,
                 o_due_eligible_bits=0, o_due_delivered_bits=0,
                 a_current_eligible_bits=0, a_available_bits=0,
                 o_current_eligible_bits=0, o_available_bits=0,
                 a_sample_eligible_bits=0, a_sample_generated_bits=0,
                 o_sample_eligible_bits=0, o_sample_generated_bits=0)
        self.current = f; self.frames.append(f)
        previous_sample = sensor._last_step
        self.timed('sense_update', self.original['update'], sensor, t, step,
                   ctrl_tick, rp, obs, sp, sv)
        self.obs = obs
        if not self.detailed:
            return
        # Everything below is read-only, after the online timed region.
        apos = np.linalg.norm(sp[None, :, :]-np.asarray(rp)[:, None, :], axis=2)
        ae = apos <= sensor.r_animal
        oa = np.array([o.is_alive() for o in obs], dtype=bool)
        op = np.array([o.pos for o in obs]).reshape(-1, 2)
        oe = (np.linalg.norm(op[None, :, :]-np.asarray(rp)[:, None, :], axis=2)
              <= sensor.r_obs)&oa[None, :]
        f['a_current_eligible_bits'] = bits(ae.any(axis=0))
        f['o_current_eligible_bits'] = bits(oe.any(axis=0))
        truth = np.full(self.k, np.inf)
        if np.any(oa):
            vel = np.array([o.vel for o in obs]).reshape(-1, 2)
            radii = np.array([self.vc.r_sheep+o.radius for o in obs])
            truth[oa] = min_cv_ttc(sp, sv, op[oa], vel[oa], radii[oa])
        f['truth_ttc'] = truth
        if sensor._last_step != previous_sample:
            ag = np.zeros(ae.shape, dtype=bool)
            og = np.zeros(oe.shape, dtype=bool)
            for j, dq in sensor._buf_ani.items():
                if dq and abs(dq[-1][0]-t) < EPS:
                    ag[j, dq[-1][1]] = True
            for (j, k), dq in sensor._buf_obs.items():
                if dq and abs(dq[-1][0]-t) < EPS:
                    og[j, k] = True
            assert not np.any(ag&~ae) and not np.any(og&~oe)
            f.update(source_t=float(t), a_eligible=int(ae.any(axis=0).sum()),
                a_generated=int(ag.any(axis=0).sum()),
                o_eligible=int(oe.any(axis=0).sum()), o_generated=int(og.any(axis=0).sum()),
                a_link_eligible=int(ae.sum()), a_link_generated=int(ag.sum()),
                o_link_eligible=int(oe.sum()), o_link_generated=int(og.sum()),
                a_sample_eligible_bits=bits(ae.any(axis=0)), a_sample_generated_bits=bits(ag.any(axis=0)),
                o_sample_eligible_bits=bits(oe.any(axis=0)), o_sample_generated_bits=bits(og.any(axis=0)))
            self.pending.append((t+sensor.delay, float(t),
                                 f['a_sample_eligible_bits'], f['o_sample_eligible_bits']))

    def publish(self, sensor, truth, t):
        v = self.timed('sense_publish', self.original['publish'], sensor, truth, t)
        if not len(v.sheep_pos) and sensor._ctrl_tick:
            self.current['tag'] = self.last_tag = 'unobserved'
        if not self.detailed:
            return v
        f = self.current
        animal_ids = np.flatnonzero(v.mask_shared)
        a_stamps = np.full(len(v.mask_shared), np.nan)
        a_stamps[animal_ids] = t-v.sheep_age
        o_stamps = np.full(self.k, np.nan)
        o_available = np.zeros(self.k, dtype=bool)
        observed = np.full(self.k, np.inf)
        if v.obs_active_n:
            ids = np.array([o.index for o in v.obstacles_active], dtype=int)
            o_stamps[ids] = [o.t_stamp for o in v.obstacles_active]
            o_available[ids] = True
            observed[ids] = min_cv_ttc(v.sheep_pos, v.sheep_vel,
                v.obs_pos, v.obs_vel,
                np.array([self.vc.r_sheep+o.radius for o in v.obstacles_active]))
        f.update(a_available_bits=bits(v.mask_shared), o_available_bits=bits(o_available),
                 animal_source_stamps=a_stamps, obstacle_source_stamps=o_stamps,
                 observed_ttc=observed)
        while self.pending and self.pending[0][0] <= t+EPS:
            _, source_t, ab, ob = self.pending.popleft()
            assert f['a_due_eligible_bits']==0 and f['o_due_eligible_bits']==0, 'Only one source sample due per integration tick'
            ad = bits(np.isfinite(a_stamps)&(np.abs(a_stamps-source_t)<EPS))&ab
            od = bits(np.isfinite(o_stamps)&(np.abs(o_stamps-source_t)<EPS))&ob
            f.update(a_due_eligible_bits=ab, a_due_delivered_bits=ad,
                     o_due_eligible_bits=ob, o_due_delivered_bits=od,
                     a_due=bit_count(ab), a_delivered=bit_count(ad),
                     o_due=bit_count(ob), o_delivered=bit_count(od))
        return v

    def controller(self, controller, original, view, cfg):
        cmd, tag = self.timed('controller', original, view, cfg)
        digest_values(self.controller_commands, (cmd, np.array(tag)))
        self.current['tag'] = self.last_tag = tag
        if tag in ('pre', 'divert'):
            core, _, _ = unpack(controller)
            if self.method == 'Ours_rollout':
                target = getattr(core, '_threat_obj', None)
            elif self.method == 'B2':
                target = threat_obstacle(view, view.centroid)
            else:
                target = None
            if target is not None:
                self.current['policy_target'] = int(target.index)
        return cmd, tag

    def fleet(self, robot, *args, **kwargs):
        return self.timed('fleet_total', self.original['fleet'], robot, *args, **kwargs)

    def guard(self, *args, **kwargs):
        return self.timed('guard', self.original['guard'], *args, **kwargs)

    def feasibility(self, *args, **kwargs):
        return self.timed('feasibility', self.original['feas'], *args, **kwargs)

    def rs(self, *args, **kwargs):
        return self.timed('rs_safety', self.original['rs'], *args, **kwargs)

    def projection(self, cmd, *args, **kwargs):
        out = self.timed('projection', self.original['project'], cmd, *args, **kwargs)
        digest_values(self.commands, (cmd,))
        return out

    def contact(self, mon, *args, **kwargs):
        out = self.original['contact'](mon, *args, **kwargs)
        for key, when in mon.new_events:
            if key[0] == 'so':
                k = int(key[2])
                self.first_contact[k] = min(self.first_contact.get(k, np.inf), when)
        return out

    def run(self, t_max=None):
        ctrl = make_controller(self.method, ALGS, self.vc, dict(self.sc.get('params', {})))
        ctrl_original = ctrl.step
        ctrl.step = lambda view, cfg: self.controller(ctrl, ctrl_original, view, cfg)
        # Bound functions do not bind the target class automatically.
        with patch.object(RobustSensorium, 'update', lambda obj,*a,**kw:self.update(obj,*a,**kw)), \
             patch.object(RobustSensorium, 'publish', lambda obj,*a,**kw:self.publish(obj,*a,**kw)), \
             patch.object(RobotFleetV2, 'step', lambda obj,*a,**kw:self.fleet(obj,*a,**kw)), \
             patch.object(FeasibilityLayer, 'filter', lambda obj,*a,**kw:self.feasibility(obj,*a,**kw)), \
             patch.object(RSSafety, 'filter', lambda obj,*a,**kw:self.rs(obj,*a,**kw)), \
             patch.object(fleet_v2, 'apply_robot_safety_v2', self.guard), \
             patch.object(fleet_v2, 'robot_step_project', self.projection), \
             patch.object(ContactMonitor, 'update', lambda obj,*a,**kw:self.contact(obj,*a,**kw)):
            row, res = run_cell(self.scene, self.method, self.seed, v2cfg=self.vc,
                                return_res=True, record_states=True, t_max=t_max,
                                controller=ctrl)
        row.update(res['v2'])
        row.update(scenario=self.scene, method=self.method, factor=self.factor,
            seed=self.seed, version='safe', arrival=bool(row['success']),
            joint_completion=bool(row['success'] and row['completion']))
        row = {k:v for k,v in row.items() if not isinstance(v,(dict,list,tuple))}
        metadata = dict(scenario=self.scene, method=self.method, factor=self.factor,
            seed=self.seed, trajectory_sha256=digest_trace(res['trace_states']),
            final_commands_sha256=self.commands.hexdigest(),
            controller_commands_sha256=self.controller_commands.hexdigest(),
            frame_count=len(self.frames), first_so_contact=self.first_contact,
            censored_source_samples=len(self.pending),
            censored_animal_opportunities=sum(bit_count(x[2]) for x in self.pending),
            censored_obstacle_opportunities=sum(bit_count(x[3]) for x in self.pending),
            result=row)
        return metadata

    def save(self, stem, directory='traces'):
        keys = sorted(self.frames[0])
        arrays = {key: np.array([f[key] for f in self.frames]) for key in keys}
        # Exactly represent 36-animal masks; never round them via float CSV.
        for key in arrays:
            if key.endswith('_bits'):
                arrays[key] = arrays[key].astype(np.uint64)
        np.savez_compressed(OUT/directory/(stem+'.npz'), **arrays)


def same_scalar(a, b):
    if a is None or b is None:
        return a is b
    if isinstance(a, (int,float,np.number,bool)) and isinstance(b,(int,float,np.number,bool)):
        return bool(a==b or (isinstance(a,(float,np.floating)) and
                            isinstance(b,(float,np.floating)) and np.isnan(a) and np.isnan(b)))
    return a == b


def stem_for(scene, method, factor, seed):
    return '__'.join((scene,method,factor,str(seed)))


def protocol():
    cells = [(s,m,f,seed) for s in SCENES for m in METHODS for f in FACTORS for seed in SEEDS]
    idx = np.random.default_rng(2026102907).permutation(len(cells))
    cells = [cells[int(i)] for i in idx]
    refs = [(s,m,'exact',3000) for s in SCENES for m in METHODS]
    refs += [('V2S3','Ours_rollout',f,3029) for f in ('delay_0p5','animal_pos_0p5','blackout_1p0')]
    timing_cells=[(s,m,f,3000) for s in SCENES for m in METHODS
                  for f in ('exact','animal_pos_0p5','delay_0p5','blackout_1p0')]
    tidx=np.random.default_rng(2026102908).permutation(len(timing_cells))
    timing_cells=[timing_cells[int(i)] for i in tidx]
    return dict(status='frozen_before_measurement', cells=cells, parity_references=refs,
        timing_cells=timing_cells,
        source_sha256=source_hash(), observer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        measurement_replays=132, independent_serial_timing_replays=24, existing_performance_N_unchanged=2340,
        event_jobs=8, serial_latency_jobs=1, BLAS_threads=1, seeds=list(SEEDS),
        scheduling_amendment='Initial serial acquisition precheck retained separately; final event acquisition is parallel and all its wall timings excluded from latency inference. The independent serial latency panel starts only after the worker pool closes. No metric or controller changes.',
        selection='first and last evaluation seed by index, fixed before measurement; all 11 factors, methods and scenes',
        sampling_endpoint='range-eligible unique target/source-sample; matching source timestamp actually published when nominal delay matures',
        censoring='source samples whose nominal receipt is after run end are right-censored, not misses',
        source_link_endpoint='eligible robot-target samples generated into sensor buffers, before fleet union',
        availability_endpoint='currently range-eligible targets with a nonexpired published track; cached old tracks allowed',
        event_endpoint='one potential encounter per obstacle release, retained even if no truth TTC<=12 reference event occurs',
        reference='first control tick with physical constant-velocity disc TTC<=12; frozen projected contact time t_ref+TTC_ref is not an actual collision time',
        information_warning='first sensor publication with delivered point estimates yielding TTC<=12, matched by obstacle ID within [t_ref-12, reference_projected_contact]; not an alarm implemented by B1/B2',
        policy_response='first actual controller pre/divert request attributed to this obstacle within same event window; B1 not implemented, not a missed alarm',
        actual_contact_lead='actual first swept animal-obstacle contact time minus first matched publication/policy warning; defined only for contacts and retained late or missing explicitly',
        latency='wall and CPU nanoseconds around actual online calls; additive outer blocks update+publish+controller+fleet; inner phases for attribution only, never double-counted',
        deadlines='50 ms integration step and 100 ms two-step controller cycle; first 10 cycles marked warmup for timing summaries',
        limits=['two seeds per cell is a measurement sample, not re-estimation of the 30-seed primary effects',
                'serial latency panel uses seed 3000 and four prespecified conditions; other conditions have no uncontended latency inference',
                'software timing on this host, not sensor acquisition/network service time or hardware WCET',
                'outer fleet timing includes read-only clock/hash recorder overhead; nonnegative upper-instrumented timing, no overhead subtraction',
                'analytical reference remains a local constant-velocity counterfactual, not future animal truth'])


def smoke():
    # Analytic edge cases, not tests that merely mirror the wrapper code.
    out=min_cv_ttc(np.array([[10.,0.]]),np.array([[-1.,0.]]),
                   np.zeros((1,2)),np.zeros((1,2)),np.array([1.]))
    assert out[0]==9.
    assert min_cv_ttc(np.array([[.5,0.]]),np.zeros((1,2)),np.zeros((1,2)),np.zeros((1,2)),np.array([1.]))[0]==0.
    assert np.isinf(min_cv_ttc(np.array([[10.,0.]]),np.array([[1.,0.]]),np.zeros((1,2)),np.zeros((1,2)),np.array([1.]))[0])
    results=[]
    for factor in ('exact','delay_0p5','loss_0p10','blackout_1p0'):
        p=ReadOnlyProbe('V2S3','Ours_rollout',factor,3000,True)
        a=p.run(t_max=1.2)
        q=ReadOnlyProbe('V2S3','Ours_rollout',factor,3000,False)
        b=q.run(t_max=1.2)
        for key in ('trajectory_sha256','final_commands_sha256','controller_commands_sha256'):
            assert a[key]==b[key],(factor,key)
        for f in p.frames:
            assert f['a_delivered']<=f['a_due'] and f['o_delivered']<=f['o_due']
        results.append(dict(factor=factor,exact_trajectory_and_commands=True,
            animal_due=sum(f['a_due'] for f in p.frames),
            animal_delivered=sum(f['a_delivered'] for f in p.frames),
            animal_censored=a['censored_animal_opportunities']))
    for label, over in (
        ('forced_100pct_loss', dict(sense_loss=1.)),
        ('forced_full_blackout', dict(sense_blackout=10.,sense_blackout_period=10.,sense_blackout_start=0.))):
        p=ReadOnlyProbe('V2S3','Ours_rollout','exact',3000,True)
        p.vc=replace(p.vc,**over)
        a=p.run(t_max=1.2)
        due=sum(f['a_due'] for f in p.frames)
        received=sum(f['a_delivered'] for f in p.frames)
        assert due>0 and received==0,(label,due,received)
        results.append(dict(factor=label,animal_due=due,animal_delivered=received,
                            expected_miss_fraction=1.))
    (OUT/'smoke_validation.json').write_text(json.dumps(dict(analytic_cases=3,replays=results),indent=2))
    print('SMOKE PASS', flush=True)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--smoke',action='store_true')
    args=ap.parse_args()
    OUT.mkdir(exist_ok=True);(OUT/'traces').mkdir(exist_ok=True);(OUT/'timing_traces').mkdir(exist_ok=True)
    frozen=json.loads((ROOT/'results/formal.manifest.json').read_text())['source_sha256']
    assert source_hash()==frozen
    if args.smoke:
        smoke();return
    plan=protocol(); pp=OUT/'protocol.json'
    if pp.exists():
        old=json.loads(pp.read_text())
        assert old==json.loads(json.dumps(plan)), 'Measurement observer or protocol changed; use a separate output directory'
    else:
        pp.write_text(json.dumps(plan,indent=2))
    aff=sorted(os.sched_getaffinity(0)) if hasattr(os,'sched_getaffinity') else []
    chosen=aff[0] if aff else None
    environment=dict(platform=platform.platform(),python=sys.version,numpy=np.__version__,
        visible_cpus=os.cpu_count(),original_affinity=aff,pinned_cpu=chosen,
        execution='events: 8 workers; timing: independent serial process pinned to one CPU after pool closes; BLAS/OMP/MKL=1',perf_counter=time.get_clock_info('perf_counter')._asdict() if hasattr(time.get_clock_info('perf_counter'),'_asdict') else str(time.get_clock_info('perf_counter')))
    (OUT/'environment.json').write_text(json.dumps(environment,indent=2))
    formal=[json.loads(x) for x in (ROOT/'results/formal.jsonl').read_text().splitlines()]
    lookup={(r['scenario'],r['method'],r['factor'],r['seed']):r for r in formal if r['version']=='safe'}
    refs_path=OUT/'parity_references.json'
    refs=json.loads(refs_path.read_text()) if refs_path.exists() else {}
    for s,m,f,seed in plan['parity_references']:
        stem=stem_for(s,m,f,seed)
        if stem not in refs:
            p=ReadOnlyProbe(s,m,f,seed,False)
            refs[stem]=p.run()
            refs_path.write_text(json.dumps(refs,indent=2))
            print('REFERENCE',stem,flush=True)
    done={}
    result_path=OUT/'runs.jsonl'
    if result_path.exists():
        for line in result_path.read_text().splitlines():
            r=json.loads(line)
            if not r.get('error'):done[r['stem']]=r
    start=time.time()
    def accept(meta,kind='events'):
        stem=meta['stem'];s,m,f,seed=(meta[k] for k in ('scenario','method','factor','seed'))
        try:
            existing=lookup[(s,m,f,seed)]
            excluded={'ctrl_ms','wall_s','error'}
            mismatches=[k for k,v in meta['result'].items() if k in existing and k not in excluded and not same_scalar(v,existing[k])]
            assert not mismatches,(stem,mismatches)
            meta.update(stem=stem,error=None,formal_scalar_parity=True,
                        formal_fields_compared=len(set(meta['result'])&set(existing)-excluded))
            if stem in refs:
                keys=('trajectory_sha256','final_commands_sha256','controller_commands_sha256')
                assert all(meta[k]==refs[stem][k] for k in keys),stem
                meta['full_trajectory_command_parity']=True
            path=result_path if kind=='events' else OUT/'timing_runs.jsonl'
            meta['measurement_kind']=kind
            with path.open('a') as fp:
                fp.write(json.dumps(meta,default=lambda x:x.item() if isinstance(x,np.generic) else str(x))+'\n')
                fp.flush();os.fsync(fp.fileno())
            return meta
        except Exception:
            error=dict(stem=stem,error=traceback.format_exc())
            (OUT/'error.json').write_text(json.dumps(error,indent=2))
            raise
    todo=[cell for cell in plan['cells'] if stem_for(*cell) not in done]
    with ProcessPoolExecutor(max_workers=8) as pool:
        futures={pool.submit(measure_event_cell,cell):cell for cell in todo}
        for future in as_completed(futures):
            meta=accept(future.result())
            done[meta['stem']]=meta
            print('EVENTS',len(done),'/',len(plan['cells']),meta['stem'],'elapsed_s',round(time.time()-start),flush=True)
    # No measurement workers remain when serial latency acquisition begins.
    if chosen is not None:os.sched_setaffinity(0,{chosen})
    timed={}
    timed_path=OUT/'timing_runs.jsonl'
    if timed_path.exists():
        for line in timed_path.read_text().splitlines():
            r=json.loads(line);timed[r['stem']]=r
    for cell in plan['timing_cells']:
        stem=stem_for(*cell)
        if stem in timed:continue
        p=ReadOnlyProbe(*cell,True)
        meta=p.run();meta['stem']=stem
        p.save(stem,'timing_traces')
        timed[stem]=accept(meta,'serial_timing')
        print('TIMING',len(timed),'/',len(plan['timing_cells']),stem,'elapsed_s',round(time.time()-start),flush=True)
    assert source_hash()==frozen
    summary=dict(status='complete',measurement_replays=len(done),errors=0,
        serial_timing_replays=len(timed),
        formal_scalar_parity_runs=sum(r['formal_scalar_parity'] for r in done.values()),
        full_trajectory_and_command_parity_runs=sum(r.get('full_trajectory_command_parity',False) for r in done.values()),
        frozen_source_sha256=frozen,source_unchanged=True,
        formal_data_sha256=hashlib.sha256((ROOT/'results/formal.jsonl').read_bytes()).hexdigest(),
        wall_s=time.time()-start)
    (OUT/'execution_status.json').write_text(json.dumps(summary,indent=2))
    print('FINISHED',json.dumps(summary),flush=True)


def measure_event_cell(cell):
    p=ReadOnlyProbe(*cell,True)
    meta=p.run();meta['stem']=stem_for(*cell)
    p.save(meta['stem'])
    return meta


if __name__=='__main__':main()
