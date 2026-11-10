"""Read-only diagnostics. Ground-truth values below NEVER enter controller.step."""
import numpy as np
from ttc import ttc_matrix


class AuditController:
    def __init__(self, inner):
        self.inner = inner
        self.history = []
        self._indices = {}
        self._contacts = set()

    def __getattr__(self, key):
        return getattr(self.inner, key)

    def step(self, view, cfg):
        cmd, tag = self.inner.step(view, cfg)
        # Diagnostics are computed AFTER the decision; no feedback of truth.
        T, _, _ = ttc_matrix(view.sheep_pos, view.sheep_vel,
                            view.obs_pos, view.obs_vel, cfg.r_sheep+cfg.r_obs)
        risk = T.min(axis=0).tolist() if len(T) else []
        threat = getattr(self.inner, '_threat_obj', None)
        ident = lambda o: f'{o.t_spawn:g}:{o.start[0]:g}:{o.start[1]:g}'
        active = [ident(o) for o in view.obstacles_active]
        tracked = ident(threat) if threat is not None else None
        contacts = []
        for k, o in enumerate(view.obstacles_active):
            hits = np.where(np.linalg.norm(view.sheep_pos_global-o.pos,axis=1)
                            < cfg.r_sheep+o.radius)[0].tolist()
            contacts.append(hits)
        self.history.append(dict(t=float(view.t), tag=tag,
            active=active, risk=risk, tracked=tracked,
            side=getattr(self.inner,'event_side',None),
            planned_offset=getattr(self.inner,'_offset',None),
            primary=list(getattr(self.inner,'primary_idx',[])),
            contacts=contacts,
            activation=[o.t0_active for o in view.obstacles_active],
            sheep=view.sheep_pos_global.tolist(),
            robots=view.robot_pos.tolist(),
            obstacles=[o.pos.tolist() for o in view.obstacles_active],
            controller_risk=float(getattr(self.inner,'_ttc_last',np.inf))))
        return cmd, tag
