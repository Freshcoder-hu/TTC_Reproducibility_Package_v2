"""V2 (Stage 1) parameter set: one frozen record, one content hash.

WHY A SEPARATE MODULE
---------------------
The V1 tree (`config.py`, `simulation.py`, `robots.py`, `sheep.py`, ...) must stay
bit-reproducible, so nothing here imports-into or mutates it.  `config.py` keeps
its job (it supplies the *simulation-level* `SimCfg`); this module adds the
parameters V2 needs that `SimCfg` has no field for (body radius, lateral
acceleration, hard separation distances, detector/feasibility numerics) and
records the V1 values beside them for the audit table.

`V2Cfg` is frozen (`dataclass(frozen=True)`) and carries `config_id`, a
truncated SHA-256 of the canonical JSON of every parameter *including* the
derived geometric distances.  Every run writes `config_id` into its row, so a
result can never be detached from the parameter set that produced it.

The three author-fixed values (`r_robot`, `a_lat`, `v_max`, plus `a_lin` and
`omega_max` which were re-fixed at the same time) are marked `AUTHOR` below and
in :data:`PARAM_TABLE`; nothing in the code base overrides them.
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass, replace

VERSION = "v2-stage2"

# The author's proposed robot-robot soft target, kept addressable for the
# documented sensitivity comparison (it is not the default; see V2Cfg).
D_RR_SAFE_AUTHOR = 1.2

# ---------------------------------------------------------------------------
# Stage 2: animal behaviour models.  Kept as module constants (not only as
# dataclass defaults) so the manuscript's wording, the report table and the
# code all cite one spelling.
# ---------------------------------------------------------------------------
ANIMAL_MODELS = ("M0", "M1", "M2")
PREDICTOR_ENGINES = ("dll", "v2")
SPEED_POLICIES = ("off", "manoeuvre")

# ---------------------------------------------------------------------------
# V1 reference values, transcribed from config.py (never imported at run time:
# this is an audit record, so that a later edit of config.py cannot silently
# rewrite the "V1" column of the report).
# ---------------------------------------------------------------------------
V1_VALUES = {
    "dt": 0.05,
    "ctrl_period": 2,
    "n_sheep": 36,
    "n_robots": 12,
    "r_sheep": 0.35,
    "r_obs": 1.2,
    "sheep_v_base": 1.0,
    "sheep_v_flee": 1.5,
    "sheep_turn_max": 3.0,
    "sheep_accel_max_declared": 4.0,
    "sheep_accel_imposed": False,
    "robot_r_body": None,          # not modelled
    "robot_v_max": 2.6,
    "robot_omega_max": 3.5,
    "robot_a_lin": 3.0,
    "robot_a_ang_declared": 5.0,
    "robot_a_ang_imposed": False,
    "robot_a_lat": None,           # not modelled
    "d_rr_safe_soft": 2.0,
    "d_ro_safe_soft": 1.2,         # == r_obs  -> "satisfied target" == touching
    "contact_rr": 0.0,
    "contact_ro": 1.2,             # == r_obs  (no robot body)
    "contact_rs": 0.35,
    "contact_ss": 0.70,
    "contact_so": 1.55,
    "slot_gap": 0.3,
    "rr_gain": 8.0,
}


@dataclass(frozen=True)
class V2Cfg:
    """Frozen V2 Stage-1 parameter record (see module docstring)."""

    version: str = VERSION

    # ---------------- author-fixed (do not change) ----------------
    r_robot: float = 0.45          # AUTHOR  disc footprint radius [m]
    a_lat: float = 2.0             # AUTHOR  lateral acceleration cap [m/s^2]
    v_max: float = 4.5             # AUTHOR  robot speed cap [m/s]
    a_lin: float = 4.0             # AUTHOR  robot linear acceleration [m/s^2]
    omega_max: float = 3.5         # AUTHOR  robot turn rate [rad/s]
    a_ang: float = 5.0             # AUTHOR  angular acceleration [rad/s^2]
    #   a_ang = 5.0 rad/s^2: config.py already declares a_ang = 5.0 (R4), so the
    #   "choose 5.0 unless the existing config suggests otherwise" instruction
    #   resolves to keeping the declared value and finally *imposing* it.
    #   5.0 rad/s^2 reaches the 3.5 rad/s rate cap in 0.70 s -- slow enough that
    #   R4 binds visibly, fast enough that heading control is not crippled.

    # ---------------- flock (unchanged radii, C3 now imposed) ----------------
    r_sheep: float = 0.35
    r_obs: float = 1.2
    s_min: float = 1.0             # C1 lower bound of the *M0* animal
    s_max: float = 1.5             # C1 upper bound (v_flee)
    omega_sheep: float = 3.0       # C2
    a_sheep: float = 4.0           # C3, now imposed on the velocity *vector*
    #   Stage-2 hook: the manuscript's (V2) box allows s_min = 0 ("admissible
    #   lower speed becomes zero to permit slowing and stopping") for M1/M2.
    #   Stage 1 keeps the M0 value 1.0 so that the only animal-side change with
    #   respect to V1 is that C3 is enforced.

    # ---------------- spacing: soft targets, decoupled from geometry --------
    d_rr_safe: float = 2.0         # soft robot-robot target
    d_ro_safe: float = 2.0         # soft robot-obstacle target (author value)
    #   AUTHOR-PROPOSED d_rr_safe was 1.2 m; it is kept as a configurable
    #   variant (see D_RR_SAFE_AUTHOR) but is NOT the default, because:
    #     * 1.2 - 0.90 = 0.30 m of margin is less than the distance a robot
    #       covers in one 0.1 s control period at v_max (0.45 m), so the target
    #       cannot be held by a command-level filter;
    #     * measured (S3, seeds 200-202, Ours_rollout): 1.2 m -> 36.7
    #       robot-robot contact episodes per run, worst surface clearance
    #       -0.46 m (hard limit BREACHED); 1.5 m -> 8 episodes, -0.36 m;
    #       2.0 m -> 0.7 episodes, +0.02 m (held).
    #     * 2.0 m equals V1's soft target, so the V1 -> V2 difference isolates
    #       the *physical* layer (body radius, caps, contact detection) instead
    #       of also changing the formation pitch (which follows d_rr_safe and
    #       would fall from 2.3 m to 1.5 m at 1.2 m).
    #   Margin over the hard limits with these values: 1.10 m (robot-robot),
    #   0.35 m (robot-obstacle).  Neither is a high-speed guarantee -- see the
    #   braking-distance analysis in the report (D_stop(4.5 m/s) = 2.53-3.38 m).

    # ---------------- scenario-level (unchanged) ----------------
    n_sheep: int = 36
    n_robots: int = 12
    slot_gap: float = 0.3

    # ---------------- limit switches (all on in V2) ----------------
    enforce_c3: bool = True
    enforce_r4: bool = True
    enforce_lat: bool = True
    enforce_hard_separation: bool = True

    # ---------------- continuous contact detection ----------------
    contact_mode: str = "chord"    # "chord": exact linear sweep per dt
    #                                "arc"  : sub-sampled unicycle arcs
    contact_substeps: int = 8      # only used by contact_mode="arc"
    contact_tol: float = 0.0       # clearance <= -tol counts as penetration

    # ---------------- feasibility layer ----------------
    feas_enabled: bool = True
    feas_pred_substeps: int = 4    # prediction samples inside one control period
    feas_act_margin: float = 0.0   # extra margin before acting [m]

    # ---- braking-distance supervisor (reviewer comment 3) ----------------
    brake_enabled: bool = True
    brake_classes: tuple = ("rr",)
    #    Robot-robot only.  A stopping-distance rule is right when *both* parties
    #    can stop; against a non-cooperative crossing obstacle, stopping in its
    #    path is the worst action.  The S3 diagnostic (v2/diag_v2.py) confirms
    #    it: braking against obstacles made the robot-obstacle surface clearance
    #    worse (-1.49 m with RO braking vs -0.94 m without), because the robot
    #    stops instead of leaving the swept corridor.  Obstacle avoidance is
    #    therefore a prediction + lateral-escape problem, handled by the
    #    hard-separation response and (Stage 2) by the controller.
    #    Robot-animal separation is deliberately NOT braking-supervised either:
    #    herding *requires* deliberate approach to animals (they flee at
    #    r_avo = 4 m), so a stopping-distance rule against animals would forbid
    #    the task itself.
    a_dec_plan: float = 3.0        # de-rated deceleration used for planning
    #    3.0 m/s^2 is 75 % of the plant's a_lin = 4.0, so the required distance
    #    is never tuned to the limit actually being reached.
    t_delay: float = 0.1           # control delay = one control period (0.1 s)
    brake_margin: float = 0.1      # residual surface margin beyond d_body

    # ---- post-hoc control-failure (C) diagnosis -------------------------
    c_diag_enabled: bool = True
    c_diag_window: int = 12        # control ticks scanned backwards (1.2 s)

    # ---------------- numerics ----------------
    eps_v: float = 1e-6            # near-zero speed threshold [m/s]
    bind_rtol: float = 1e-9        # "did this bound clip?" comparison slop
    viol_tol: float = 1e-6         # violation accounting tolerance

    # ==================================================================
    # Stage 2 -- animal behaviour models M0 / M1 / M2
    # ==================================================================
    #   Defaults reproduce Stage 1 exactly (M0, no extra layer), so
    #   ``V2Cfg()`` stays the inert configuration and ``default_v2cfg()``
    #   still reproduces the Stage-1 archive bit-for-bit.  The shipped
    #   Stage-2 configuration is built by :func:`stage2_v2cfg`.
    animal_model: str = "M0"       # M0 | M1 | M2 (model the simulator integrates)

    # ---- (V2) box band of the behaviour model -------------------------
    #   `s_min`/`s_max` above remain the *V1* C1 band (mirrored to SimCfg and
    #   used by the legacy V1 kinematic check), so the meaning of every V1
    #   column is unchanged.  The model's own admissible band is separate and
    #   is what the projection enforces for M1/M2.
    m_s_min: float = 0.0           # admissible lower speed of the model (0 => can stop)
    m_v_base: float = 1.0          # normal speed reference [m/s] (manuscript 1.0-1.5)
    m_v_flee: float = 1.5          # robot-flee speed [m/s]
    m_v_max: float = 2.0           # stimulus-dependent upper reference [m/s] (1.8-2.0 exploratory)

    # ---- M1: local, finite obstacle avoidance -------------------------
    #   Calibrated on seeds 1000-1019 (v2/calibrate_v2.py): linear decay,
    #   k_M1 = 3.0 at the mid sensing range; see calibration_m1.csv.
    m1_sense_range: float = 6.0    # animal-local obstacle range, on the SURFACE GAP [m]
    m1_gain: float = 3.0           # response gain k_M1 [same scale as rep_dir]
    m1_decay: str = "linear"       # linear | quadratic | exp
    m1_decay_len: float = 2.0      # e-folding length of the "exp" shape [m]
    m1_obs_delay: float = 0.10     # animal-local observation delay [s] (2 ticks at dt=0.05)
    m1_body_guard: float = 0.0     # surface gap at which pure body separation wins [m]

    # ---- M2: behavioural-stress layer on top of M1 --------------------
    #   M2 is a *stress* model, not a validated description of frightened
    #   sheep; see the report.
    m2_gain_sigma: float = 0.20    # lognormal sigma of the per-animal response gain
    m2_react_delay: float = 0.30   # extra per-animal reaction delay [s] (added to m1_obs_delay)
    m2_stim_ref: float = 2.0       # stimulus value mapped to the upper speed reference
    m2_slow_gain: float = 0.5      # speed factor at a 90 deg turn demand (1 = no slowing)
    m2_speed_map: bool = True      # stimulus-dependent speed target (False => M1 rule)
    m2_seed_tag: int = 20260227    # stream tag mixed with the run seed

    # ---- predictor / environment decoupling ---------------------------
    #   The rollout predictor may use a different animal model from the one
    #   the plant integrates.  `predictor_engine="dll"` keeps the V1 native
    #   predictor (M0 only); `"v2"` uses this package's NumPy rollout, which
    #   accepts M0/M1/M2.
    predictor_model: str = "M0"
    predictor_engine: str = "dll"
    predictor_shadow: bool = False  # evaluate + log predictions, do not actuate
    predictor_speed_policy: bool = True
    predictor_horizon: float = 12.0
    predictor_dt: float = 0.15

    # ---- layered speed policy (controller layer, not a physical limit) --
    #   v_max applies to straight-line repositioning only; while
    #   pre-positioning or diverting the commanded speed is bounded by
    #   v <= sqrt(a_lat * r_req) with r_req derived from the commanded
    #   lateral offset and the path geometry.
    speed_policy: str = "off"      # off | manoeuvre
    speed_policy_modes: tuple = ("pre", "divert")
    speed_policy_h_eps: float = 0.05   # |h| below this => straight => no bound [m]

    # ---- provenance ----------------
    #   The *soft* spacing targets live in SimCfg as well (the V1 helpers read
    #   them from there); the module-level SIM_MIRROR below defines the
    #   one-to-one mapping, so a V2Cfg override always reaches the plant and no
    #   second hard-coded list can drift.

    # ------------------------------------------------------------------
    # derived geometry -- the single source of truth for contact distances
    # ------------------------------------------------------------------
    @property
    def d_rr_contact(self) -> float:
        """robot-robot surface contact (centre distance): 2 r_robot."""
        return 2.0 * self.r_robot

    @property
    def d_ro_contact(self) -> float:
        """robot-obstacle surface contact: r_robot + r_o."""
        return self.r_robot + self.r_obs

    @property
    def d_rs_contact(self) -> float:
        """robot-sheep surface contact: r_robot + r_s."""
        return self.r_robot + self.r_sheep

    @property
    def d_ss_contact(self) -> float:
        """sheep-sheep surface contact: 2 r_s."""
        return 2.0 * self.r_sheep

    @property
    def d_so_contact(self) -> float:
        """sheep-obstacle surface contact: r_s + r_o."""
        return self.r_sheep + self.r_obs

    def hard_contacts(self) -> dict:
        return {"rr": self.d_rr_contact, "ro": self.d_ro_contact,
                "rs": self.d_rs_contact, "ss": self.d_ss_contact,
                "so": self.d_so_contact}

    # ------------------------------------------------------------------
    # hashing / serialisation
    # ------------------------------------------------------------------
    def payload(self) -> dict:
        d = {k: v for k, v in asdict(self).items()}
        d["derived"] = self.hard_contacts()
        d["v1_reference"] = dict(V1_VALUES)
        d["sim_mirror"] = dict(SIM_MIRROR)
        return d

    def canonical_json(self) -> str:
        return json.dumps(self.payload(), sort_keys=True,
                          separators=(",", ":"), ensure_ascii=True)

    @property
    def config_sha256(self) -> str:
        return hashlib.sha256(self.canonical_json().encode("utf-8")).hexdigest()

    @property
    def config_id(self) -> str:
        """Short, human-quotable identifier (first 12 hex of the SHA-256)."""
        return self.config_sha256[:12]

    # ------------------------------------------------------------------
    def with_(self, **kw) -> "V2Cfg":
        return replace(self, **kw)

    def assert_consistent(self, sim_cfg) -> None:
        """Fail loudly if the SimCfg a run uses disagrees with this record.

        The *soft* spacing targets live in ``SimCfg`` because the V1 helpers
        (``robots.apply_robot_safety``, ``robots.slot_positions``) read them from
        there.  They are mirrored here for tracing, so the mirror must be exact.
        """
        pairs = [("d_rr_safe", self.d_rr_safe), ("d_ro_safe", self.d_ro_safe),
                 ("v_max", self.v_max), ("a_lin", self.a_lin),
                 ("omega_max", self.omega_max), ("a_ang", self.a_ang),
                 ("r_sheep", self.r_sheep), ("r_obs", self.r_obs),
                 ("n_sheep", self.n_sheep), ("n_robots", self.n_robots)]
        bad = [(k, getattr(sim_cfg, k, None), v) for k, v in pairs
               if getattr(sim_cfg, k, None) != v]
        if bad:
            raise ValueError(
                "V2Cfg / SimCfg mismatch (soft targets must be mirrored): "
                + ", ".join(f"{k}: sim={a!r} v2={b!r}" for k, a, b in bad))


def default_v2cfg(**overrides) -> V2Cfg:
    return replace(V2Cfg(), **overrides)


# V2Cfg field -> SimCfg field (the plant-level mirror; one source of truth)
SIM_MIRROR = {
    "v_max": "v_max",
    "a_lin": "a_lin",
    "a_ang": "a_ang",
    "omega_max": "omega_max",
    "d_rr_safe": "d_rr_safe",
    "d_ro_safe": "d_ro_safe",
    "a_sheep": "accel_max",
    "omega_sheep": "turn_max",
    "s_min": "v_base",
    "s_max": "v_flee",
    "r_sheep": "r_sheep",
    "r_obs": "r_obs",
    "n_sheep": "n_sheep",
    "n_robots": "n_robots",
    "slot_gap": "slot_gap",
}


def sim_overrides(v2cfg: V2Cfg) -> dict:
    """SimCfg key/values implied by ``v2cfg`` (always in sync)."""
    return {SIM_MIRROR[k]: getattr(v2cfg, k) for k in SIM_MIRROR}


def compat_v1cfg(**overrides) -> V2Cfg:
    """The inertia configuration for acceptance test A.

    r_robot -> 0, the four new/imposed caps disabled, the feasibility layer
    bypassed.  Everything else keeps its V1 value; the *caller* must also pass
    the archived V1 ``SimCfg`` (v_max 2.6, a_lin 3.0, d_rr_safe 2.0,
    d_ro_safe 1.2, ...) so that the V1 helpers see V1 numbers.
    """
    base = dict(r_robot=0.0, enforce_c3=False, enforce_r4=False,
                enforce_lat=False, enforce_hard_separation=False,
                feas_enabled=False, contact_mode="chord",
                animal_model="M0", speed_policy="off", predictor_engine="dll")
    base.update(overrides)
    return V2Cfg(**base)


def stage2_v2cfg(animal_model="M2", **overrides) -> V2Cfg:
    """The shipped Stage-2 configuration (Stage-1 physical layer + behaviour).

    ``animal_model`` selects the behaviour model the *simulator* integrates.
    Every other Stage-1 value is untouched, so an M0 run of this configuration
    is bit-identical to Stage 1 and the M1/M2 contrast isolates the animal
    response.
    """
    base = dict(animal_model=str(animal_model), speed_policy="manoeuvre",
                predictor_engine="dll", predictor_model="M0")
    base.update(overrides)
    return V2Cfg(**base)


# ---------------------------------------------------------------------------
# the report's parameter table (V1 | V2 | note), generated from the record so
# that the document can never drift from the code
# ---------------------------------------------------------------------------
PARAM_TABLE = [
    # (key, V1, V2, note)
    ("r_robot [m]", "not modelled", "0.45", "AUTHOR-FIXED; disc footprint"),
    ("a_lat [m/s^2]", "not modelled", "2.0", "AUTHOR-FIXED; |v*omega| cap"),
    ("v_max [m/s]", "2.6", "4.5", "AUTHOR-FIXED"),
    ("a_lin [m/s^2]", "3.0", "4.0", "AUTHOR-FIXED"),
    ("omega_max [rad/s]", "3.5", "3.5", "AUTHOR-FIXED; unchanged"),
    ("a_ang [rad/s^2]", "5.0 declared, not imposed", "5.0 imposed",
     "author: 5.0 unless config suggests otherwise; config.py already 5.0"),
    ("d_rr_contact [m]", "0.0", "0.90", "derived 2*r_robot; V1 had no body"),
    ("d_ro_contact [m]", "1.20 (= r_o)", "1.65", "derived r_robot + r_o"),
    ("d_rs_contact [m]", "0.35 (= r_s)", "0.80", "derived r_robot + r_s"),
    ("d_ss_contact [m]", "0.70", "0.70", "derived 2*r_s"),
    ("d_so_contact [m]", "1.55", "1.55", "derived r_s + r_o"),
    ("d_rr_safe (soft) [m]", "2.00", "2.00",
     "margin 1.10 over the 0.90 hard limit; author proposed 1.2 (=0.30 margin,"
     " rejected with data - see D2)"),
    ("d_ro_safe (soft) [m]", "1.20 (= r_o)", "2.00",
     "margin 0.35 over the 1.65 hard limit; author value kept"),
    ("slot_gap [m]", "0.30", "0.30", "unchanged; slot pitch follows d_rr_safe"),
    ("r_sheep [m]", "0.35", "0.35", "unchanged"),
    ("r_obs [m]", "1.2", "1.2", "unchanged"),
    ("n_sheep : n_robots", "36 : 12", "36 : 12", "unchanged"),
    ("C1 sheep speed [m/s]", "1.0-1.5 (imposed)", "1.0-1.5 (imposed)",
     "upper bound preserved by the C3 projection"),
    ("C2 sheep turn [rad/s]", "3.0 (imposed)", "3.0 (imposed)", "unchanged"),
    ("C3 sheep accel [m/s^2]", "4.0 declared, NOT imposed", "4.0 imposed on the "
     "velocity vector", "V1 jumped 1.0->1.5 m/s in one step (= 10 m/s^2)"),
    ("dt [s]", "0.05", "0.05", "unchanged; common integration step"),
    ("ctrl_period [ticks]", "2", "2", "unchanged (0.1 s)"),
]


def param_table_markdown() -> str:
    head = ("| parameter | V1 | V2 (Stage 1) | note |\n"
            "|---|---|---|---|\n")
    return head + "\n".join(f"| {a} | {b} | {c} | {d} |"
                            for a, b, c, d in PARAM_TABLE)


# ---------------------------------------------------------------------------
# Stage-2 parameter table (rendered from a V2Cfg instance, so the document
# cannot drift from the code -- same principle as PARAM_TABLE)
# ---------------------------------------------------------------------------
STAGE2_TABLE = [
    # (field, label, searched range / provenance, note)
    ("animal_model", "animal model integrated by the simulator",
     "M0 / M1 / M2",
     "M0 = Stage-1 model (bit-exact); M1 = +local obstacle term; M2 = +stress"),
    ("m1_sense_range", "M1 animal obstacle sensing range [m] (surface gap)",
     "sweep 3 / 6 / 9; shipped 6",
     "M1 weight vanishes for gap >= R by construction"),
    ("m1_gain", "M1 avoidance gain k_M1",
     "searched 0.5 / 1.0 / 2.0 / 3.0; calib. 3.0",
     "same scale as rep_dir (sum of unit vectors)"),
    ("m1_decay", "M1 distance-decay shape",
     "searched linear / quadratic / exp(1 m) / exp(3 m); calib. linear",
     "parsimony tie-break, see V2_STAGE2_REPORT 4"),
    ("m1_decay_len", "exp-decay e-folding length [m]",
     "1.0 / 3.0 (only for exp)", "unused by the shipped linear shape"),
    ("m1_obs_delay", "animal-local observation delay [s]",
     "fixed 0.10 (= 2 ticks)", "delayed observation, no shared tracks"),
    ("m1_body_guard", "surface gap at which body separation wins [m]",
     "fixed 0.0", "obstacle term dropped on body overlap"),
    ("m_s_min", "admissible lower speed of the behaviour model [m/s]",
     "0 (author decision)", "allows slowing and stopping; M0 keeps 1.0"),
    ("m_v_base", "normal speed reference [m/s]", "fixed 1.0", "manuscript 1.0-1.5"),
    ("m_v_flee", "robot-flee speed [m/s]", "fixed 1.5", "unchanged from V1"),
    ("m_v_max", "stimulus-dependent upper reference [m/s]",
     "searched 1.5 / 1.8 / 2.0; shipped 2.0",
     "exploratory 1.8-2.0 per the manuscript"),
    ("m2_gain_sigma", "lognormal sigma of the per-animal gain",
     "searched 0.10 / 0.20 / 0.30; shipped 0.20",
     "mean renormalised to 1 (same total gain as M1)"),
    ("m2_react_delay", "extra per-animal reaction delay [s]",
     "searched 0.15 / 0.30 / 0.45; shipped 0.30",
     "added to m1_obs_delay on the observed stimulus"),
    ("m2_stim_ref", "stimulus mapped to m_v_max",
     "fixed 2.0", "saturating speed map"),
    ("m2_slow_gain", "speed factor at a 90 deg turn demand",
     "searched 1.0 / 0.5 / 0.0; shipped 0.5",
     "design override: makes the admissible 0 reachable"),
    ("m2_seed_tag", "RNG stream tag mixed with the run seed",
     "fixed", "gains/delays identical for a given seed"),
    ("predictor_model", "animal model used by the rollout predictor",
     "M0 / M1 / M2; default M0", "independent of animal_model"),
    ("predictor_engine", "predictor implementation",
     "dll (V1, M0 only) / v2 (NumPy, M0-M2)", "dll keeps Stage-1 exactly"),
    ("predictor_shadow", "evaluate predictions without actuating them",
     "False / True", "used to prove the decoupling"),
    ("predictor_speed_policy", "predictor obeys the speed policy",
     "True / False", "controller rule, so the prediction must obey it"),
    ("speed_policy", "layered speed policy (controller layer)",
     "off / manoeuvre", "not a physical limit"),
    ("speed_policy_modes", "modes in which the bound applies",
     "pre, divert", "straight-line repositioning keeps v_max"),
    ("speed_policy_h_eps", "|h| below which the manoeuvre counts as straight [m]",
     "fixed 0.05", "r_req = inf => v_allow = v_max"),
]


def param_table_stage2_markdown(cfg: "V2Cfg" = None) -> str:
    cfg = cfg or V2Cfg()
    head = ("| parameter | shipped value | searched range / provenance | note |\n"
            "|---|---|---|---|\n")
    rows = []
    for field, label, rng, note in STAGE2_TABLE:
        val = getattr(cfg, field)
        if isinstance(val, tuple):
            val = ",".join(str(v) for v in val)
        rows.append(f"| {label} | `{field}` = {val} | {rng} | {note} |")
    return head + "\n".join(rows)


if __name__ == "__main__":
    cfg = V2Cfg()
    print("config_id      :", cfg.config_id)
    print("config_sha256  :", cfg.config_sha256)
    print("hard contacts  :", cfg.hard_contacts())
    print()
    print(param_table_markdown())
    print()
    print("Stage-2 shipped configuration "
          f"(config_id {stage2_v2cfg().config_id}):")
    print(param_table_stage2_markdown(stage2_v2cfg()))
