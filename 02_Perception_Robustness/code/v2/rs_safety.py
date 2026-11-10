"""Native finite-candidate observation-only safety filter; no truth access.
This reduces predicted overlap; it is NOT a collision-avoidance theorem.
"""
import ctypes
from pathlib import Path
import numpy as np

class RSSafety:
    def __init__(self,cfg,v2cfg):
        self.cfg=cfg;self.c=v2cfg
        self.lib=ctypes.CDLL(str(Path(__file__).resolve().parent.parent/'_rs_safety.so'))
        self.fun=self.lib.rs_select
        self.fun.restype=None
        self.changed=self.no_safe=self.calls=self.empty=0
    def filter(self,cmd,robot,sp,sv,sa,op,ov,oa):
        self.calls+=1
        if not len(sp):
            self.empty+=1
            return np.zeros_like(cmd)
        c=self.c;cfg=self.cfg
        pars=[cfg.dt,cfg.v_max,cfg.omega_max,cfg.a_lin,c.a_ang,c.a_lat,
              c.rs_horizon,c.r_robot+c.r_sheep,2*c.r_robot,c.r_robot+c.r_obs,
              c.rs_margin,c.sense_animal_err,c.sense_err,c.rs_sigma_scale,c.rs_growth]
        arr=[robot.pos,robot.head,robot.speed,robot.omega,cmd,sp,sv,sa,op,ov,oa,pars]
        arr=[np.ascontiguousarray(x,dtype='float64') for x in arr]
        out=np.empty_like(cmd);stats=np.empty((len(cmd),3))
        ptr=lambda a:a.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        self.fun(ctypes.c_int(len(cmd)),ctypes.c_int(len(sp)),ctypes.c_int(len(op)),
                 *map(ptr,arr),ptr(out),ptr(stats))
        self.changed+=int(np.sum(np.linalg.norm(out-cmd,axis=1)>1e-9))
        self.no_safe+=int(np.sum(stats[:,1]==0))
        return out
    def summary(self):
        return dict(rs_safety_calls=self.calls,rs_safety_changed=self.changed,
                    rs_no_safe_candidate=self.no_safe,rs_empty_stop_ticks=self.empty)
