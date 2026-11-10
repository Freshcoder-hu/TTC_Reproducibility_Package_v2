import os
os.environ['OPENBLAS_NUM_THREADS']='1'
import sys,json,ctypes
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'code'))
from v2.config_v2 import stage3_v2cfg
from v2.limits_v2 import robot_step_project
from v2.robust_sensing import RobustSensorium,ObservedView
from v2.scenarios_v2 import resolve_scene,cfg_for
from simulation import View
from obstacle import Obstacle

rng=np.random.default_rng(7331)
lib=ctypes.CDLL(str(Path(__file__).resolve().parents[1]/'code/_rs_safety.so'))
fun=lib.test_rs_step;fun.restype=None
DP=ctypes.POINTER(ctypes.c_double)
p=np.array([.05,4.5,3.5,4.,5.,2.])
maxerr=0.
for _ in range(2000):
 v=float(rng.uniform(0,4.5));w=float(rng.uniform(-min(3.5,2/max(v,.001)),min(3.5,2/max(v,.001))))
 h=float(rng.uniform(-6,6));cmd=rng.normal(size=2)*3
 ref=robot_step_project(cmd,v,h,w,dt=.05,v_max=4.5,omega_max=3.5,a_lin=4.,a_ang=5.,a_lat=2.,ang_on=True,lat_on=True)[:3]
 a,b,c=ctypes.c_double(v),ctypes.c_double(h),ctypes.c_double(w)
 fun(ctypes.c_double(cmd[0]),ctypes.c_double(cmd[1]),ctypes.byref(a),ctypes.byref(b),ctypes.byref(c),p.ctypes.data_as(DP))
 maxerr=max(maxerr,float(np.max(np.abs(np.array(ref)-[a.value,b.value,c.value]))))
assert maxerr<1e-11,maxerr

base=stage3_v2cfg(robust_strict=True,sense_assert=True,sense_delay=.5)
sc=resolve_scene('V2S2R2');cfg=cfg_for(sc,base)
s=RobustSensorium(base,cfg,8100)
v=View();v.path=sc['path'];v.robot_pos=np.array([[0.,0.]])
v.sheep_pos_global=np.array([[3.,0.],[40.,0.]])
v.sheep_vel_global=np.array([[1.,0.],[1.,0.]])
v.obs_pos=np.zeros((0,2));v.obs_vel=np.zeros((0,2))
for step in range(13):
 t=step*.05
 s.update(t,step,True,v.robot_pos,[],v.sheep_pos_global,v.sheep_vel_global)
 out=s.publish(v,t)
 if t<.5-1e-9:assert len(out.sheep_pos)==0
 else:
  assert len(out.sheep_pos)==1
  assert out.sheep_age[0]>=.5-1e-9
  assert not hasattr(out,'sheep_pos_global') and not hasattr(out,'sheep')
# Poison unseen truth only after the sensor sampled: publication cannot change.
before=out.sheep_pos.copy()
v.sheep_pos_global[:]=999999.
out2=s.publish(v,.6)
assert np.array_equal(before,out2.sheep_pos)
# No observations never causes a truth fallback.
s2=RobustSensorium(stage3_v2cfg(robust_strict=True,sense_r_animal=.01),cfg,8100)
s2.update(0,0,True,v.robot_pos,[],v.sheep_pos_global,v.sheep_vel_global)
assert len(s2.publish(v,0).sheep_pos)==0 and s2._n_fallback==0
r=dict(native_projection_cases=2000,native_projection_max_abs_error=maxerr,
       delayed_track_survives_0p5s=True,empty_observation_no_truth=True,
       poisoned_unseen_truth_cannot_change_publication=True)
print(json.dumps(r,indent=2))
Path(__file__).with_suffix('.json').write_text(json.dumps(r,indent=2))
