"""仿真实验主程序（论文 5 节）。

默认（快速，覆盖 RQ1-RQ4）：
    python run_experiments.py --seeds 6
论文正式实验（RQ1-RQ6，>=30 seeds，含 PRE 消融）：
    python run_experiments.py --all --seeds 30 --ablation
    python run_experiments.py --scenarios S2R1 S2R2 S2R3 S4S S4M S4L --seeds 30

对比方法：B1 原始沿路径牧羊 / B2 距离阈值反应式 / B3 动态危险反应式 /
          Ours（TTC→PRE→DIVERT→RECOVER + 角色协调）/
          Ours_w/o_PRE（消融，--ablation）
输出：results/detail.csv（每 seed 明细）, results/summary.csv（均值±标准差）
"""
import argparse
import csv
import os
import time
from collections import defaultdict, Counter

import numpy as np

from scenarios import SCENARIOS, MAIN_SCENARIOS, QUICK_SCENARIOS
from robots import NormalHerder, DistanceThreshHerder, CagingHerder
from baselines_vh import VHPathHerder, VHDangerHerder
from baselines_outmost import BOutmostHerder
from baselines_pcbf import BPCBHerder
from proposed import Proposed
from simulation import run_scene
from metrics import summarize, fmt


class ProposedNoPRE(Proposed):
    """PRE 消融：TTC 直接进入 DIVERT（TTC→DIVERT→RECOVER）。"""
    name = "Ours_w/o_PRE"

    def step(self, view, cfg):
        old = self.t_pre
        self.t_pre = self.t_divert
        try:
            return super().step(view, cfg)
        finally:
            self.t_pre = old


ALGS = {
    "B1": lambda **kw: NormalHerder(),
    "B2": lambda **kw: DistanceThreshHerder(),
    "B3": lambda **kw: CagingHerder(r_detect=15.0, cage=0.8),
    "Ours": lambda **kw: Proposed(**{k: v for k, v in kw.items()
                                     if k in ("divert_shift",
                                              "pre_shift_ratio", "t_pre",
                                              "t_divert")}),
    "Ours_w/o_PRE": lambda **kw: ProposedNoPRE(**{k: v for k, v in kw.items()
                                                  if k in ("divert_shift",
                                                           "pre_shift_ratio",
                                                           "t_pre",
                                                           "t_divert")}),
    # 外部同行方法（忠实复现，见 BASELINES_SPEC.md）
    "VH-Path": lambda **kw: VHPathHerder(),
    # VH-Danger 的两种官方 scenario 模式（原文一次只跑一种）：
    #   VH-Danger      = mixed（危险块方向 + 路径任务回退）—— 其危险响应机制生效
    #   VH-Danger-path = path （原文 scenario == "path"）—— 同一条控制律的最强任务变体
    "VH-Danger": lambda **kw: VHDangerHerder(task_mode="mixed"),
    "VH-Danger-path": lambda **kw: VHDangerHerder(task_mode="path"),
    # B-Outmost（T-RO 2024；作者未开源 → 基于原文公式的机制级复现，见 §7）
    "B-Outmost": lambda **kw: BOutmostHerder(),
    # B-PCBF（RA-L 2026；作者未开源 → 机制级复现，见 §8）
    "B-PCBF": lambda **kw: BPCBHerder(),
}

KEYS = [
    # 安全性
    "collision_rate", "min_dist", "dmin_robot_obs", "dmin_robot_robot",
    "rr_min_t", "rr_min_tag", "rr_min_pair",
    "ttc_min", "ttc_margin", "ttc_p5",
    # 路径与凝聚
    "rho_avg", "rho_min", "max_dev", "cflock_avg", "cflock_min",
    # 恢复与控制
    "recovery", "recovery_success", "switch_count", "role_switches",
    "control_var", "coverage_direct", "coverage_shared",
    "violation_rate", "ctrl_ms",
    # 结果与离群 + 时序证据
    "success", "out_normal", "out_risk",
    "t_pre_on", "t_divert_on", "t_dist_trigger", "t_risk_on", "ttc_lead",
    "dur_pre", "dur_divert",
    # 附录乙表（Nguyen 2026 指标口径，全方法统一复算）
    "exec_time", "completion", "d_goal_end", "dist_range_end",
    "head_err_mean",
]

# 论文主表只看这些（核心列）；其余仅在 summary.csv(附录) 中保留
CORE_KEYS = [
    "collision_rate",        # 防碰撞：最硬安全指标
    "min_dist",              # 羊—障碍物最小距离
    "dmin_robot_robot",      # 机器人间最小距离（协调安全）
    "ttc_min",               # 最危险时刻的 TTC
    "ttc_p5",                # TTC 5% 分位
    "t_risk_on",             # TTC 首次达到 T_PRE 的时刻
    "t_divert_on",           # DIVERT 触发时刻
    "t_dist_trigger",        # 距离阈值法会触发的时刻（同场景标定）
    "ttc_lead",              # >0 表示 TTC 早于距离阈值触发
    "success",               # 任务完成
    "max_dev",               # 最大路径偏离
    "rho_avg",               # 路径保持/凝聚
    "rho_min",
    "out_risk",              # 风险相关离群
    "recovery_success",      # RECOVER 成功率
    "switch_count",          # 状态切换
    "role_switches",         # 角色切换
    "control_var",           # 控制平滑度
    "ctrl_ms",               # 单步计算开销
]

CONSOLE_KEYS = CORE_KEYS


def run_task(sid, alg, seeds, seed_offset, gate=None):
    """单个 (场景, 方法) 的全部种子运行 —— 模块级函数，可被进程池 pickle。

    独立性保证（串行/并行结果必须逐位一致）：
      * `run_scene` 内部用 `np.random.default_rng(seed)`，不依赖全局 RNG；
      * `run_scene` 会**深拷贝**场景障碍物（`simulation.py` 的 `obs = [Obstacle(...)]`），
        因此同一场景对象被多个方法复用时不会被污染；
      * 控制器对象在每个 seed 重新构造，无跨运行状态。

    `gate = (radius, use_fov)` 或 None：障碍物探测门（`sensing_gate.py`）。
    门控必须在**每个 worker 进程内**安装（`run_scene` 在调用时通过
    `simulation` 模块全局名解析 `View`）；`radius=None` 时完全不安装，
    行为与加门之前逐位相同。
    """
    import sensing_gate
    gate_r, gate_fov = (None, False) if gate is None else (gate[0], gate[1])
    sensing_gate.install(gate_r, gate_fov)
    scn = SCENARIOS[sid]()
    acc = defaultdict(list)
    d_rows = []
    for k in range(seeds):
        seed = seed_offset + k
        ctrl = sensing_gate.wrap(ALGS[alg](**scn.get("params", {})))
        res = run_scene(scn["cfg"], scn["path"], scn["obstacles"], ctrl,
                        seed=seed, T_max=scn["T_max"], s0=scn["s0"])
        m = summarize(res)
        for kk in KEYS:
            acc[kk].append(m.get(kk, np.nan))
        d_rows.append([sid, scn["name"], alg, seed] +
                      [m.get(kk, np.nan) for kk in KEYS])
    mean = {kk: stat(acc[kk])[0] for kk in KEYS}
    std = {kk: stat(acc[kk])[1] for kk in KEYS}
    return dict(sid=sid, name=scn["name"], alg=alg, detail=d_rows,
                summary=[sid, scn["name"], alg,
                         *[mean[kk] for kk in KEYS],
                         *[std[kk] for kk in KEYS]],
                mean=mean)


def _task_entry(task):
    """进程池入口（必须是模块级函数）。"""
    return run_task(*task)


def stat(values):
    """数值列返回 (mean, std)；非数值列（如 rr_min_tag/pair）返回众数。"""
    try:
        arr = np.asarray(values, dtype=float)
    except (ValueError, TypeError):
        vals = [v for v in values if v not in (None, "", "?")]
        if not vals:
            return "?", "?"
        mode = Counter(vals).most_common(1)[0][0]
        return mode, "?"
    finite = arr[np.isfinite(arr)]
    if len(finite) == 0:
        return np.nan, np.nan
    return float(finite.mean()), float(finite.std())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenarios", nargs="+", default=QUICK_SCENARIOS,
                    choices=list(SCENARIOS.keys()))
    ap.add_argument("--all", action="store_true",
                    help="运行论文全部场景（S1/S1N/S2/S2R*/S3/S4*）")
    ap.add_argument("--seeds", type=int, default=6,
                    help="随机种子数（正式实验建议 >=30，且各方法配对使用）")
    ap.add_argument("--seed-offset", type=int, default=0,
                    help="种子起点：评估用 0（默认）；参数标定请用 100 "
                         "（约定标定集 100-104，评估集 0-29，互不重叠）")
    ap.add_argument("--algs", nargs="+",
                    default=["B1", "B2", "B3", "Ours"],
                    choices=list(ALGS.keys()))
    ap.add_argument("--ablation", action="store_true",
                    help="加入 Ours_w/o_PRE 消融（5.12）")
    ap.add_argument("--jobs", type=int, default=1,
                    help="并行进程数：1=串行（默认）；>1 按 (场景,方法) 并行。"
                         "每个任务自带 seed、run_scene 内部自建 RNG 且深拷贝障碍物，"
                         "因此并行结果与串行逐位一致（见 run_task 文档）")
    ap.add_argument("--out", default="results")
    ap.add_argument("--sensing-radius", type=float, default=None, metavar="R",
                    help="障碍物探测门半径 R [m]（sensing_gate.py）：只有队内"
                         "任一机器人在 R 内时控制器才看到该障碍物。缺省 None"
                         "= 不装门（与加门之前逐位相同）。")
    ap.add_argument("--gate-fov", action="store_true",
                    help="额外要求障碍物落在该机器人的感知扇区内（v4 谱系的"
                         "原语义）。注意：开此项目标 R→∞ 不再逐位惰性。")
    args = ap.parse_args()
    args.jobs = max(1, int(args.jobs))
    gate = (None, False) if args.sensing_radius is None \
        else (float(args.sensing_radius), bool(args.gate_fov))

    scenarios = MAIN_SCENARIOS if args.all else args.scenarios
    algs = list(args.algs)
    if args.ablation and "Ours_w/o_PRE" not in algs:
        algs.append("Ours_w/o_PRE")

    os.makedirs(args.out, exist_ok=True)
    t0 = time.time()
    detail_rows, summary_rows = [], []

    print("=" * 96)
    if args.jobs > 1:
        print("[注意] --jobs > 1：`ctrl_ms`（墙钟计时）会受进程负载影响；"
              "论文的每周期耗时须来自独立串行跑批（PREREGISTRATION §9）。"
              "其余指标与串行逐位一致（verify_parallel.py）。")
        print("-" * 96)
    print("TTC Shepherding — simulation experiments (draft Chapter 5)")
    print("Scenarios :", ", ".join(scenarios))
    print("Algorithms:", ", ".join(algs))
    print("Seeds     :", args.seeds, "(paired across methods)")
    print("Jobs      :", args.jobs, "(1 = 串行；>1 = 按 (场景,方法) 并行，"
          "结果与串行逐位一致)")
    if gate[0] is None:
        print("Sensing   : obstacle gate OFF (full visibility, frozen behaviour)")
    else:
        print(f"Sensing   : obstacle detection gate R = {gate[0]:g} m"
              f"{' + FOV cone' if gate[1] else ' (range-only)'}")
    print("=" * 96)

    tasks = [(sid, alg, args.seeds, args.seed_offset, gate)
             for sid in scenarios for alg in algs]
    if args.jobs > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=args.jobs) as ex:
            results = list(ex.map(_task_entry, tasks))       # 保持顺序
    else:
        results = [_task_entry(t) for t in tasks]

    prev_sid = None
    for r in results:
        if r["sid"] != prev_sid:
            print(f"\n===== {r['sid']}: {r['name']} =====")
            prev_sid = r["sid"]
        detail_rows.extend(r["detail"])
        summary_rows.append(r["summary"])
        print(f"[{r['alg']:12s}] " + " | ".join(
            f"{k}={fmt(r['mean'][k])}" for k in CONSOLE_KEYS), flush=True)

    with open(os.path.join(args.out, "detail.csv"), "w", newline="",
              encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["scenario", "scenario_name", "alg", "seed"] + KEYS)
        w.writerows(detail_rows)
    with open(os.path.join(args.out, "summary.csv"), "w", newline="",
              encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["scenario", "scenario_name", "alg"] +
                   [f"{k}_mean" for k in KEYS] +
                   [f"{k}_std" for k in KEYS])
        w.writerows(summary_rows)

    # 主表（论文正文用）：只含 CORE_KEYS
    core_idx = [KEYS.index(k) for k in CORE_KEYS]
    with open(os.path.join(args.out, "summary_core.csv"), "w", newline="",
              encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["scenario", "scenario_name", "alg"] +
                   [f"{k}_mean" for k in CORE_KEYS] +
                   [f"{k}_std" for k in CORE_KEYS])
        for row in summary_rows:
            means = row[3:3 + len(KEYS)]
            stds = row[3 + len(KEYS):3 + 2 * len(KEYS)]
            w.writerow(row[:3] + [means[i] for i in core_idx] +
                       [stds[i] for i in core_idx])

    print("\n===== 场景等权汇总 =====")
    by_alg = defaultdict(lambda: defaultdict(list))
    for row in summary_rows:
        alg = row[2]
        means = row[3:3 + len(KEYS)]
        for i, k in enumerate(KEYS):
            by_alg[alg][k].append(means[i])

    def agg(alg, k):
        a = np.asarray(by_alg[alg][k], dtype=float)
        a = a[np.isfinite(a)]
        return float(np.mean(a)) if len(a) else np.nan

    head = ["Alg"] + CORE_KEYS
    print("| " + " | ".join(head) + " |")
    print("|" + "---|" * len(head))
    for alg in algs:
        print(f"| {alg} | " + " | ".join(
            fmt(agg(alg, k)) for k in CORE_KEYS) + " |")

    print(f"\n结果已保存：{args.out}/detail.csv（明细）, summary_core.csv"
          f"（论文主表）, summary.csv（附录全指标）")
    print("统计分析/绘图：python analyze_results.py --out "
          f"{args.out} --figs")
    print(f"总耗时：{time.time() - t0:.2f} s")


if __name__ == "__main__":
    main()
