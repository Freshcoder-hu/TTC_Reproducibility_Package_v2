"""Check native predictions against the independent original Python dynamics."""
import unittest
from types import SimpleNamespace
from dataclasses import replace
import numpy as np
from config import default_cfg
from path import straight_path
from obstacle import Obstacle
from sheep import init_flock, SheepState, advance_sheep
from robots import RobotFleet, slot_positions, flock_geom, drive_command, _norm1
from proposed_rollout import RolloutProposed


class PredictionTests(unittest.TestCase):
    def test_candidate_horizon_cannot_falsely_clear_recovery(self):
        from proposed_event import EventProposed
        cfg=default_cfg(n_robots=12)
        path=straight_path(0,110)
        class ObservedObstacle:
            pos=np.array([8.+1.8*16.8+1.55,0.])
            vel=np.array([-.8,0.])
            radius=1.2
            def is_alive(self):return True
        o=ObservedObstacle()
        view=SimpleNamespace(t=10.,path=path,sheep_pos=np.array([[8.,0.]]),
            sheep_vel=np.array([[1.,0.]]),centroid=np.array([8.,0.]),
            robot_pos=np.zeros((12,2)),obs_pos=np.array([o.pos]),
            obs_vel=np.array([o.vel]),obstacles_active=[o],obs_active_n=1)
        ctrl=EventProposed();ctrl.state='recover';ctrl._threat_obj=o;ctrl._event_threat=o
        risk,_,_=ctrl._ttc(view,cfg)
        self.assertAlmostEqual(risk,16.8)
        _,tag=ctrl.step(view,cfg)
        self.assertEqual(tag,'recover')

    def test_controller_requires_no_hidden_sheep_or_future_obstacle_fields(self):
        cfg=default_cfg(n_robots=12)
        path=straight_path(0,110)
        st=init_flock(36,path.point_at(8),radius=2.8,rng=np.random.default_rng(100))
        slots,_=slot_positions(st.pos.mean(axis=0),3.,np.array([1.,0.]),cfg,form='ring')
        class ObservedObstacle:
            pos=np.array([20.,3.])
            vel=np.array([-.8,-.2])
            radius=1.2
            def is_alive(self):return True
        o=ObservedObstacle()
        view=SimpleNamespace(t=0.,path=path,sheep_pos=st.pos,sheep_vel=st.vel,
            centroid=st.pos.mean(axis=0),robot_pos=slots,obs_pos=np.array([o.pos]),
            obs_vel=np.array([o.vel]),obstacles_active=[o],obs_active_n=1)
        ctrl=RolloutProposed()
        command,tag=ctrl.step(view,cfg)
        self.assertEqual(command.shape,(12,2))
        self.assertTrue(np.isfinite(command).all())
        self.assertTrue(ctrl.plans)

    def test_one_second_matches_original_dynamics_at_original_step(self):
        cfg=default_cfg(n_robots=12)
        path=straight_path(0,110)
        sheep=init_flock(36,path.point_at(8),radius=2.8,rng=np.random.default_rng(100))
        slots,_=slot_positions(sheep.pos.mean(axis=0),3.,np.array([1.,0.]),cfg,form='ring')
        obs=Obstacle([15,5],[-.8,-.45]);obs._activate(0)
        view=SimpleNamespace(t=0.,path=path,sheep_pos=sheep.pos,sheep_vel=sheep.vel,
             centroid=sheep.pos.mean(axis=0),robot_pos=slots,
             obs_pos=np.array([obs.pos]),obs_vel=np.array([obs.vel]),obstacles_active=[obs])
        ctrl=RolloutProposed(horizon=1.,prediction_dt=.05)
        ctrl._heading=np.zeros(12)
        targets=[-2.4,0,2.4]
        native=ctrl._predict(view,cfg,targets)
        expected=[]
        for offset in targets:
            st=SheepState(sheep.pos.copy(),sheep.head.copy(),sheep.speed.copy())
            robots=RobotFleet(slots.copy(),head=np.zeros(12),cfg=cfg)
            op=obs.pos.copy();margin=np.full(36,np.inf)
            outside=center=spread=0.
            initial=float(path.closest(st.pos.mean(axis=0))[0])
            for tick in range(20):
                v=SimpleNamespace(path=path,sheep_pos=st.pos,robot_pos=robots.pos,
                                  centroid=st.pos.mean(axis=0))
                c,r=flock_geom(v);s,_,_=path.closest(c)
                ahead=6. if offset==0 else 4.
                d=path.tangent_at(s+ahead);n=np.array([-d[1],d[0]])
                direction=_norm1(path.point_at(s+ahead)+n*offset-c)
                cmd,_,_=drive_command(v,cfg,c,r,direction,'predict')
                robots.step(cmd,cfg,np.array([op]))
                advance_sheep(st,cfg,robots.pos,path)
                op=op+obs.vel*cfg.dt
                c=st.pos.mean(axis=0);cs,cl,_=path.closest(c)
                cw=path.halfwidth_at(cs)
                center+=float(cl*cl/cw**2)
                si,li,_=path.closest(st.pos)
                width=np.array([path.halfwidth_at(x) for x in si])
                outside+=float(np.maximum(0,np.abs(li)-width+cfg.r_sheep).dot(
                                   np.maximum(0,np.abs(li)-width+cfg.r_sheep)))
                spread+=float(np.sum(np.maximum(0,np.linalg.norm(st.pos-c,axis=1)-12)**2))
                margin=np.minimum(margin,np.linalg.norm(st.pos-op,axis=1)-cfg.r_sheep-obs.radius)
            expected.append([np.mean(margin<0),np.mean(np.maximum(0,.8-margin)**2),
                             outside/(36*20),center/20,spread/(36*20),
                             float(cs)-initial,float(margin.min())])
        np.testing.assert_allclose(native,np.array(expected),rtol=1e-8,atol=1e-8)


if __name__=='__main__':
    unittest.main()
