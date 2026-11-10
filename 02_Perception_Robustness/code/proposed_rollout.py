"""Receding-horizon action selection using only available observations.

The native library is a prediction model, never an alternative scoring simulator.
Robot heading/speed are estimated from successive positions, not hidden truth.
No future spawn times, obstacle starts, seeds or scenario names enter decisions.
"""
import ctypes
from pathlib import Path
import subprocess
import numpy as np
from proposed_event import EventProposed
from robots import flock_geom, drive_command, _norm1


class RolloutProposed(EventProposed):
    name = 'Ours_rollout'

    def __init__(self, horizon=12., prediction_dt=.15, plan_period=1., **kw):
        super().__init__(**kw)
        self.horizon, self.prediction_dt = float(horizon),float(prediction_dt)
        self.plan_period = float(plan_period)
        self._plan_t = -np.inf
        self._offset = 0.
        self._previous_pos = None
        self._previous_t = None
        self._heading = None
        self.plans = []
        root=Path(__file__).parent
        lib=root/('_rollout.dll' if __import__('sys').platform.startswith('win') else '_rollout.so')
        # Build once before a parallel run (runner does this in parent).
        if not lib.exists():
            raise SystemExit('build _rollout.dll first')
        self._lib=ctypes.CDLL(str(lib))
        self._evaluate=self._lib.evaluate_rollouts
        self._evaluate.restype=None

    def _predict(self, view, cfg, offsets):
        p=np.ascontiguousarray(view.sheep_pos,dtype='float64')
        v=np.ascontiguousarray(view.sheep_vel,dtype='float64')
        rp=np.ascontiguousarray(view.robot_pos,dtype='float64')
        speed=np.zeros(len(rp))
        if self._previous_pos is not None and view.t>self._previous_t:
            velocity=(rp-self._previous_pos)/(view.t-self._previous_t)
            speed=np.linalg.norm(velocity,axis=1)
            moving=speed>1e-4
            self._heading[moving]=np.arctan2(velocity[moving,1],velocity[moving,0])
        if self._heading is None:
            s,_,_=view.path.closest(view.centroid)
            d=view.path.tangent_at(s)
            self._heading=np.full(len(rp),np.arctan2(d[1],d[0]))
        op=np.ascontiguousarray(view.obs_pos,dtype='float64')
        ov=np.ascontiguousarray(view.obs_vel,dtype='float64')
        radii=np.array([o.radius for o in view.obstacles_active],dtype='float64')
        pars=np.array([cfg.v_base,cfg.v_flee,cfg.turn_max,cfg.r_rep,cfg.r_ori,
            cfg.r_att,cfg.r_avo,cfg.avo_k,cfg.goal_bias,cfg.v_max,cfg.omega_max,
            cfg.a_lin,cfg.kp,cfg.d_rr_safe,cfg.d_ro_safe,cfg.slot_gap,cfg.r_sheep,4.],dtype='float64')
        actions=np.ascontiguousarray(offsets,dtype='float64')
        output=np.zeros((len(actions),7),dtype='float64')
        arrays=[p,v,rp,self._heading,speed,op,ov,radii,
                np.ascontiguousarray(view.path.verts),np.ascontiguousarray(view.path.halfwidth),pars,actions]
        ptr=lambda a:a.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        self._evaluate(*[ctypes.c_int(n) for n in (len(p),len(rp),len(op),len(view.path.verts),len(actions))],
                       *[ptr(a) for a in arrays],ctypes.c_double(self.prediction_dt),
                       ctypes.c_int(round(self.horizon/self.prediction_dt)),ptr(output))
        return output

    def step(self, view, cfg):
        nominal,tag=super().step(view,cfg)
        if self._heading is None:
            s,_,_=view.path.closest(view.centroid)
            d=view.path.tangent_at(s)
            self._heading=np.full(len(view.robot_pos),np.arctan2(d[1],d[0]))
        if tag in ('pre','divert','recover') and view.obs_active_n:
            if view.t-self._plan_t>=self.plan_period-1e-7:
                s,_,_=view.path.closest(view.centroid)
                bound=min(3.6,.6*view.path.halfwidth_at(s))
                offsets=np.unique(np.clip([0.,-1.35,1.35,-2.4,2.4,-3.6,3.6,
                                           self._offset],-bound,bound))
                outcomes=self._predict(view,cfg,offsets)
                cost=(100*outcomes[:,0]+10*outcomes[:,1]+30*outcomes[:,2]
                      +.2*outcomes[:,3]+2*outcomes[:,4]-.05*outcomes[:,5]
                      +.02*(offsets-self._offset)**2)
                best=int(np.argmin(cost))
                self._offset=float(offsets[best])
                self._plan_t=float(view.t)
                self.plans.append(dict(t=float(view.t),offset=self._offset,
                                      offsets=offsets.tolist(),outcomes=outcomes.tolist(),cost=cost.tolist()))
            c,radius=flock_geom(view)
            s,_,_=view.path.closest(c)
            ahead=6. if abs(self._offset)<1e-8 else 4.
            tangent=view.path.tangent_at(s+ahead)
            normal=np.array([-tangent[1],tangent[0]])
            target=view.path.point_at(s+ahead)+normal*self._offset
            direction=_norm1(target-c)
            nominal,_,_=drive_command(view,cfg,c,radius,direction,tag)
        else:
            self._offset=0.
        self._previous_pos=view.robot_pos.copy()
        self._previous_t=float(view.t)
        return nominal,tag
