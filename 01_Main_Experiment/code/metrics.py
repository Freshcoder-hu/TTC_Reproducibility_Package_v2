"""实验评价指标（论文 5.13 节）。

5.13.1 碰撞率     5.13.2 最小距离（羊—障碍物 / 机器人—障碍物 / 机器人—机器人）
5.13.3 最小 TTC   5.13.4 最大路径偏离   5.13.5 羊群凝聚率（均值/最小）
5.13.6 恢复时间   5.13.7 恢复成功率     5.13.8 模式切换次数
5.13.9 机器人间最小距离   5.13.10 控制变化幅度 Δu   5.13.11 感知覆盖率
额外：PRE/DIVERT 触发时刻与持续时间、约束违例率、计算开销、
      正常离群/风险离群计数、终止原因（5.14）、角色切换次数（4.9）
"""
import numpy as np


def _recovery_from_modes(ts, tags):
    """恢复时间：首次 RECOVER 开始 -> 回到 NORMAL 的时间。"""
    if len(ts) == 0 or len(tags) == 0:
        return np.nan, False
    start = None
    for i, tg in enumerate(tags):
        if tg == "recover":
            start = i
            break
    if start is None:
        return np.nan, False
    for j in range(start + 1, len(tags)):
        if tags[j] == "normal":
            return float(ts[j] - ts[start]), True
        if tags[j] in ("divert", "pre"):
            start = j                      # 被新风险抢占，重新计时
    return np.nan, False


def _mode_stats(ts, tags):
    pre_on = divert_on = np.nan
    for tt, tg in zip(ts, tags):
        if tg == "pre" and not np.isfinite(pre_on):
            pre_on = tt
        if tg == "divert" and not np.isfinite(divert_on):
            divert_on = tt
    dtc = float(np.diff(ts).mean()) if len(ts) > 1 else 0.1
    dur = {m: (sum(1 for tg in tags if tg == m) * dtc
               if any(tg == m for tg in tags) else np.nan)
           for m in ("pre", "divert", "recover")}
    return pre_on, divert_on, dur["pre"], dur["divert"], dur["recover"]


def summarize(res, t_ignore=2.0):
    logs = res["logs"]
    n0 = res["n0"]
    ctrl = res.get("controller_obj", None)
    collision_rate = res["lost"] / max(n0, 1)

    base = dict(
        collision_rate=collision_rate, rho_avg=0.0, rho_min=0.0, max_dev=0.0,
        min_dist=np.nan, recovery=np.nan, recovery_success=False,
        ttc_margin=np.nan, ttc_p5=np.nan, ttc_min=np.nan,
        cflock_avg=0.0, cflock_min=0.0,
        dmin_robot_obs=res.get("dmin_robot_obs", np.nan),
        dmin_robot_robot=res.get("dmin_robot_robot", np.nan),
        rr_min_t=np.nan, rr_min_tag="?", rr_min_pair="?",
        coverage_direct=np.nan, coverage_shared=np.nan,
        control_var=res.get("control_var", np.nan),
        violation_rate=res.get("violation_rate", np.nan),
        ctrl_ms=res["ctrl_ms"], success=bool(res.get("success", False)),
        timeout=bool(res.get("timeout", False)),
        terminated_by=res.get("terminated_by", "?"),
        switch_count=getattr(ctrl, "switch_count", np.nan),
        role_switches=res.get("role_switches", np.nan),
        out_normal=res.get("out_normal", np.nan),
        out_risk=res.get("out_risk", np.nan),
        t_pre_on=np.nan, t_divert_on=np.nan,
        dur_pre=np.nan, dur_divert=np.nan, dur_recover=np.nan,
        t_dist_trigger=np.nan, t_risk_on=np.nan, ttc_lead=np.nan,
        recovery_events=0, recovery_success_rate=np.nan, ttc0=np.nan,
        event_risk_on_1=np.nan, event_risk_on_2=np.nan, event_risk_on_3=np.nan,
        event_clear_1=np.nan, dt12=np.nan, dt23=np.nan, event_order_ok=False,
        recover_start_1=np.nan, preempt_ok=False,
    )
    # ---- Nguyen 2026 附录乙表（原文指标口径，全方法统一复算）----
    #   exec_time : 仿真钟下**首次同时**满足原文成功判据（d_goal≤2.0 m 且
    #               d_range≤12 m）的时刻；未完成记 NaN（右删失，不用 T_max 顶替）
    #   completion: 是否完成（= exec_time 存在）
    #   d_goal_end / dist_range_end : 末态"到目标距离"/"分布范围"
    #   head_err_mean : 全程朝向误差 Δφ 均值（度）
    base.update(exec_time=np.nan, completion=False, d_goal_end=np.nan,
                dist_range_end=np.nan, head_err_mean=np.nan)
    if logs and "nguyen_ok" in logs[0]:
        ok_t = [l["t"] for l in logs if l.get("nguyen_ok")]
        base["exec_time"] = float(ok_t[0]) if ok_t else np.nan
        base["completion"] = bool(ok_t)
        last = logs[-1]
        base["d_goal_end"] = float(last.get("d_goal", np.nan))
        base["dist_range_end"] = float(last.get("d_range", np.nan))
        he = [l.get("head_err", np.nan) for l in logs]
        he = [h for h in he if h is not None and np.isfinite(h)]
        base["head_err_mean"] = float(np.mean(he)) if he else np.nan
    if not logs:
        return base

    ts = np.array([l["t"] for l in logs], dtype=float)
    rho = np.array([l["rho"] for l in logs], dtype=float)
    lat = np.array([l["lat"] for l in logs], dtype=float)
    ttc = np.array([l.get("ttc", np.inf) for l in logs], dtype=float)
    dmin = np.array([l.get("dmin", np.inf) for l in logs], dtype=float)
    cflock = np.array([l.get("cflock", np.nan) for l in logs], dtype=float)
    cov_d = np.array([l.get("cov_direct", np.nan) for l in logs], dtype=float)
    cov_s = np.array([l.get("cov_shared", np.nan) for l in logs], dtype=float)
    tags = [l.get("tag", "normal") for l in logs]

    m = ts >= t_ignore
    rho_avg = float(rho[m].mean()) if m.any() else float(rho.mean())
    rho_min = float(rho.min())
    max_dev = float(np.abs(lat[m]).max()) if m.any() else 0.0

    rec, rec_ok = _recovery_from_modes(ts, tags)
    pre_on, div_on, dur_pre, dur_div, dur_rec = _mode_stats(ts, tags)

    fin = np.isfinite(ttc) & (ttc > 0)
    ttc_margin = float(ttc[fin].mean()) if fin.any() else np.nan
    ttc_p5 = float(np.percentile(ttc[fin], 5)) if fin.any() else np.nan
    ttc_min = float(ttc[fin].min()) if fin.any() else np.nan

    dfin = np.isfinite(dmin)
    min_dist = float(dmin[dfin].min()) if dfin.any() else np.nan
    # 逐积分步统计的羊—障碍物最小距离（口径更严格）
    dmin_dt = res.get("dmin_sheep_obs", np.inf)
    if np.isfinite(dmin_dt):
        min_dist = min(min_dist, float(dmin_dt)) if np.isfinite(min_dist) \
            else float(dmin_dt)

    # ---- 恢复统计口径（5.13.7）：成功次数 / 风险事件数 ----
    rec_events = 1 if any(tg == "recover" for tg in tags) else 0
    rec_success_rate = float(rec_ok) if rec_events else np.nan

    cf_ok = np.isfinite(cflock)
    cflock_avg = float(cflock[cf_ok].mean()) if cf_ok.any() else 0.0
    cflock_min = float(cflock[cf_ok].min()) if cf_ok.any() else 0.0

    # ---- 时序证据：TTC 触发 vs 距离阈值触发（论文 6.1 需要）----
    cfg = res.get("cfg", None)
    d_trig = getattr(cfg, "b2_d_trigger", np.nan) if cfg is not None else np.nan
    t_dist_trigger = np.nan
    if np.isfinite(d_trig) and np.isfinite(dmin).any():
        hit = np.where(dmin <= d_trig)[0]
        if len(hit):
            t_dist_trigger = float(ts[hit[0]])
    t_risk_on = np.nan
    t_pre_thr = getattr(ctrl, "t_pre", None)
    if t_pre_thr is not None and np.isfinite(ttc).any():
        hit = np.where(ttc <= float(t_pre_thr))[0]
        if len(hit):
            t_risk_on = float(ts[hit[0]])
    # >0 表示 TTC 先触发、距离阈值后触发 => TTC 提前量
    ttc_lead = (t_dist_trigger - t_risk_on) \
        if (np.isfinite(t_dist_trigger) and np.isfinite(t_risk_on)) else np.nan

    # ---- 风险事件时序（论文 6.4 / 5.10：连续风险的顺序与抢占）----
    ev = {"event_risk_on": [], "event_clear": []}
    ttc_each = [l.get("ttc_each") for l in logs]
    if ttc_each and ttc_each[0]:
        K = len(ttc_each[0])
        ref = getattr(cfg, "risk_ref_ttc", 12.0) if cfg is not None else 12.0
        for k in range(K):
            col = np.array([row[k] if row else np.inf for row in ttc_each],
                           dtype=float)
            fin = np.isfinite(col) & (col < ref)
            idx = np.where(fin)[0]
            ev["event_risk_on"].append(float(ts[idx[0]]) if len(idx) else np.nan)
            clr = np.nan
            if len(idx):
                after = np.where((np.arange(len(col)) > idx[0]) &
                                 (~fin))[0]
                clr = float(ts[after[0]]) if len(after) else np.nan
            ev["event_clear"].append(clr)
    r_on = [v for v in ev["event_risk_on"]]
    dt12 = (r_on[1] - r_on[0]) if len(r_on) > 1 and np.isfinite(r_on[0]) \
        and np.isfinite(r_on[1]) else np.nan
    dt23 = (r_on[2] - r_on[1]) if len(r_on) > 2 and np.isfinite(r_on[1]) \
        and np.isfinite(r_on[2]) else np.nan
    order_ok = bool(len(r_on) >= 3 and np.isfinite(r_on[:3]).all() and
                    r_on[0] < r_on[1] < r_on[2])
    # RECOVER 窗口（用于判断"新风险是否在前一次恢复期内出现"）
    rec_start = next((ts[i] for i, tg in enumerate(tags) if tg == "recover"),
                     np.nan)
    rec_end = next((ts[j] for j in range(len(tags))
                    if tags[j] == "recover" and
                    any(tg == "normal" for tg in tags[j:])), np.nan)
    preempt_ok = bool(np.isfinite(rec_start) and np.isfinite(r_on[1]) and
                      r_on[1] >= rec_start) if len(r_on) > 1 else False

    out = dict(base)
    out.update(
        rho_avg=rho_avg, rho_min=rho_min, max_dev=max_dev,
        min_dist=min_dist, recovery=rec, recovery_success=bool(rec_ok),
        ttc_margin=ttc_margin, ttc_p5=ttc_p5, ttc_min=ttc_min,
        cflock_avg=cflock_avg, cflock_min=cflock_min,
        rr_min_t=res.get("rr_min_t", np.nan),
        rr_min_tag=res.get("rr_min_tag", "?"),
        rr_min_pair="-".join(str(x) for x in res.get("rr_min_pair", (-1, -1))),
        coverage_direct=float(np.nanmean(cov_d)) if np.isfinite(cov_d).any()
        else np.nan,
        coverage_shared=float(np.nanmean(cov_s)) if np.isfinite(cov_s).any()
        else np.nan,
        t_pre_on=pre_on, t_divert_on=div_on,
        dur_pre=dur_pre, dur_divert=dur_div, dur_recover=dur_rec,
        t_dist_trigger=t_dist_trigger, t_risk_on=t_risk_on, ttc_lead=ttc_lead,
        recovery_events=rec_events, recovery_success_rate=rec_success_rate,
        ttc0=float(ttc[0]) if len(ttc) else np.nan,
        event_risk_on_1=r_on[0] if len(r_on) > 0 else np.nan,
        event_risk_on_2=r_on[1] if len(r_on) > 1 else np.nan,
        event_risk_on_3=r_on[2] if len(r_on) > 2 else np.nan,
        event_clear_1=ev["event_clear"][0] if len(ev["event_clear"]) > 0
        else np.nan,
        dt12=dt12, dt23=dt23, event_order_ok=order_ok,
        recover_start_1=rec_start, preempt_ok=preempt_ok,
        success_t=res.get("success_t"),
    )
    return out


def fmt(v, nd=2):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return str(v)
    if not np.isfinite(v):
        return "N/A"
    return f"{v:.{nd}f}"
