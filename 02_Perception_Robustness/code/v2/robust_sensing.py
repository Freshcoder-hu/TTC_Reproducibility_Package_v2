"""Strict supplemental sensing; no animal truth in the published interface.

Range membership uses truth in the sensor simulator only. Independent fixed-shape
random arrays are drawn at EVERY sample, including for invisible/dropped targets,
so exogenous draws align by physical time across controllers. Velocities/IDs and
radii remain exact (or delayed): this is an explicitly limited position-noise test.
TTL measures time since nominal reception; source age is exposed separately.
Blackout is a shared, periodic animal-channel outage proxy, not visual rendering.
"""
from collections import deque
import numpy as np
from v2.sensing_v2 import Sensorium, ObsTrack, _EPS


class ObservedView:
    __slots__ = ('t','path','robot_pos','obstacles_active','obs_active_n',
                 'obs_pos','obs_vel','obs_age','sheep_pos','sheep_vel',
                 'sheep_age','mask_direct','mask_shared')
    @property
    def centroid(self):
        return self.sheep_pos.mean(axis=0) if len(self.sheep_pos) else np.zeros(2)


class RobustSensorium(Sensorium):
    def __init__(self, v2cfg, cfg, seed=0):
        super().__init__(v2cfg, cfg, seed)
        self.receive_ttl = self.ttl
        self.ttl = self.receive_ttl + self.delay
        self.animal_err = float(v2cfg.sense_animal_err)
        self._obs_noise = np.random.default_rng([seed, 2026102801])
        self._ani_noise = np.random.default_rng([seed, 2026102802])
        self._packets = np.random.default_rng([seed, 2026102803])
        self.phase = float(np.random.default_rng([seed,2026102804]).uniform(
            0, v2cfg.sense_blackout_period))
        self._empty_animals = 0
        self._animal_ages = []
        self._blackout_samples = 0

    def blackout(self, t):
        c = self.v2cfg
        return c.sense_blackout > 0 and t >= c.sense_blackout_start and (
            (t-c.sense_blackout_start+self.phase) % c.sense_blackout_period
            < c.sense_blackout-_EPS)

    def _sample(self, t, robot_pos, obs):
        self._n_samples += 1
        M, K = len(robot_pos), len(obs)
        noise = self._obs_noise.normal(0,self.err,(M,K,2))
        drops = self._packets.random((M,K)) < self.loss
        for k,o in enumerate(obs):
            if not o.is_alive(): continue
            ds = np.linalg.norm(np.asarray(robot_pos)-o.pos,axis=1)
            for j in np.where(ds <= self.r_obs)[0]:
                if drops[j,k]:
                    self._n_dropped += 1
                    continue
                self._buf_obs.setdefault((int(j),k),deque()).append(
                    (t,o.pos.copy()+noise[j,k],o.vel.copy(),float(o.radius),float(ds[j])))

    def _sample_animals(self,t,robot_pos,sheep_pos,sheep_vel):
        M,N = len(robot_pos),len(sheep_pos)
        noise = self._ani_noise.normal(0,self.animal_err,(M,N,2))
        drops = self._packets.random(M) < self.loss
        if self.blackout(t):
            self._blackout_samples += 1
            return
        if not M or not N: return
        d=np.linalg.norm(sheep_pos[None,:,:]-np.asarray(robot_pos)[:,None,:],axis=2)
        for j in range(M):
            if drops[j]:
                self._n_dropped += 1
                continue
            idx=np.where(d[j]<=self.r_animal)[0]
            self._buf_ani.setdefault(j,deque()).append(
                (t,idx,sheep_pos[idx].copy()+noise[j,idx],sheep_vel[idx].copy()))

    def publish(self, view_truth, t):
        v=ObservedView()
        v.t=float(t); v.path=view_truth.path
        v.robot_pos=view_truth.robot_pos.copy()
        by_k={}
        for (j,k),dq in self._buf_obs.items():
            msg=self._delivered(dq,t)
            if msg is None or t-msg[0]-self.delay > self.receive_ttl+_EPS: continue
            cur=by_k.get(k)
            if cur is None or msg[0]>cur[0] or (msg[0]==cur[0] and j<cur[1]):
                by_k[k]=(msg[0],j,msg[1],msg[2],msg[3])
        tracks=[]
        for k,(stamp,j,p,vel,rad) in sorted(by_k.items()):
            tr=self._tracks.setdefault(k,ObsTrack(k,rad))
            tr.pos=np.asarray(p).copy();tr.vel=np.asarray(vel).copy()
            tr.radius=float(rad);tr.t_stamp=float(stamp);tr.age=float(t-stamp)
            tr.source=int(j);tr.n_updates+=1
            if self.extrapolate: tr.pos += tr.vel*tr.age
            if tr.first_detect_t is None:
                tr.first_detect_t=float(stamp);self._first_track[k]=float(stamp)
            tracks.append(tr)
        v.obstacles_active=tracks;v.obs_active_n=len(tracks)
        v.obs_pos=np.array([o.pos for o in tracks]).reshape(-1,2)
        v.obs_vel=np.array([o.vel for o in tracks]).reshape(-1,2)
        v.obs_age=np.array([o.age for o in tracks])
        best={}
        for j,dq in sorted(self._buf_ani.items()):
            msg=self._delivered(dq,t)
            if msg is None or t-msg[0]-self.delay > self.receive_ttl+_EPS: continue
            stamp,idx,pos,vel=msg
            for q,i in enumerate(idx):
                prev=best.get(int(i))
                if prev is None or stamp>prev[0]:best[int(i)]=(stamp,pos[q],vel[q])
        order=sorted(best)
        v.sheep_pos=np.array([best[i][1] for i in order]).reshape(-1,2)
        v.sheep_vel=np.array([best[i][2] for i in order]).reshape(-1,2)
        v.sheep_age=np.array([t-best[i][0] for i in order])
        n_all=len(view_truth.sheep_pos_global)
        m=np.zeros(n_all,dtype=bool);m[order]=True
        v.mask_direct=m;v.mask_shared=m.copy()
        self._last_fallback=False
        if self._ctrl_tick:
            self._n_steps+=1
            self._n_empty_steps+=int(not tracks)
            self._empty_animals+=int(not order)
            self._ages.extend(v.obs_age.tolist())
            self._animal_ages.extend(v.sheep_age.tolist())
            self._ani_count.append(len(order))
            self.log.append(dict(t=float(t),n_obs=len(tracks),
                obs_index=[o.index for o in tracks],obs_age=v.obs_age.tolist(),
                obs_pos=v.obs_pos.tolist(),n_animals=len(order),animal_fallback=False))
        if self.assert_on:
            assert not hasattr(v,'sheep_pos_global')
            assert not hasattr(v,'sheep')
            for o in tracks:
                assert o.age >= self.delay-_EPS
                assert o.age-self.delay <= self.receive_ttl+_EPS
                assert self._buf_obs[(o.source,o.index)]
            assert not np.shares_memory(v.sheep_pos,view_truth.sheep_pos_global)
        return v

    def summary(self):
        d=super().summary()
        ages=np.array(self._animal_ages)
        d.update(sense_ttl=self.receive_ttl,sense_ttl_clock='reception',
                 sense_animal_err=self.animal_err,sense_blackout=self.v2cfg.sense_blackout,
                 sense_empty_animal_ticks=self._empty_animals,
                 sense_animal_age_mean=float(ages.mean()) if len(ages) else np.nan,
                 sense_animal_age_max=float(ages.max()) if len(ages) else np.nan,
                 sense_blackout_samples=self._blackout_samples,
                 truth_in_execution=False)
        return d
