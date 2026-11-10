"""Build a dated, post-evaluation acceptance addendum without changing protocol."""
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
audit = json.loads((ROOT/'analysis/information_boundary/summary.json').read_text())
assert audit['status'] == 'pass'
rows = [json.loads(s) for s in (ROOT/'results/formal.jsonl').read_text().splitlines()]
assert len(rows) == 2340 and not any(r.get('error') for r in rows)
safe = [r for r in rows if r['version']=='safe']
assert len(safe) == 1980

nominal = {}
for version in ('archive', 'strict', 'safe'):
    rs = [r for r in rows if r['version']==version and r['factor']=='exact'
          and r['method']=='Ours_rollout']
    assert len(rs) == 60
    nominal[version] = dict(n=len(rs), rs_contact_runs=sum(r['any_contact_rs'] for r in rs),
        animal_obstacle_contact_fraction=sum(r['collision_rate'] for r in rs)/len(rs),
        joint_completion=sum(r['joint_completion'] for r in rs))

coverage = dict(
    animal_obstacle_fraction='available per run; distinct animals, not run incidence',
    true_minimum_clearances='available, offline truth metrics',
    joint_completion='available for all runs including failures',
    robot_animal_contacts='available, continuous swept geometry',
    predicted_candidate_infeasibility='rs_no_safe_candidate is available; not a physical-unavoidability certificate',
    miss_rate='not validated; empty observation ticks and mean visible count are not false-negative rate with eligible-target denominator',
    warning_lead='partial legacy ttc_lead; not validated as actual delivered-warning lead for every threat event',
    control_compute_time='ctrl_ms covers controller.step only; excludes sensor fusion, feasibility and new RS supervisor',
    wall_time='wall_s recorded per run in a concurrent batch; not a serial per-cycle latency benchmark')

status = dict(
    addendum_date='2026-10-29', audit_timing='after_completed_evaluation',
    new_performance_runs_this_update=0,
    information_boundary=dict(status='pass_within_tested_scope', paired_checks=144,
        universal_proof=False, source_unchanged=audit['source_unchanged']),
    physical_safety=dict(status='residual_contacts_and_tradeoffs_remain',
        nominal_ours=nominal, all_safe_rs_contact_runs=sum(r['any_contact_rs'] for r in safe)),
    perception_matrix=dict(status='computed_but_expanded_acceptance_metrics_incomplete',
        main_runs=1980, decomposition_runs=360, new_batch_authorized_or_started=False),
    metrics_coverage=coverage,
    formal_data_sha256=hashlib.sha256((ROOT/'results/formal.jsonl').read_bytes()).hexdigest(),
    frozen_source_sha256=audit['frozen_source_sha256'],
    legacy_ttc_lead_nonfinite_runs=sum(not math.isfinite(r.get('ttc_lead',float('nan'))) for r in safe))
(ROOT/'analysis/acceptance_status.json').write_text(json.dumps(status, ensure_ascii=False, indent=2))

text = f'''TTC 补充实验：信息边界反事实验收及三阶段状态
更新日期：2026-10-29

本说明补充此前 8 页报告；如涉及“信息边界验收是否充分”，以本次新增检查为准。
本次没有追加正式性能运行，也没有修改已冻结的控制器、执行层或 2,340 条正式结果。

一、A 阶段：新增的最终执行指令反事实验收通过

此前 validate_mechanisms.py 只验证发布观测不受采样后真值变化影响，不足以
单独证明整个执行通路满足信息边界。本次新增 verify_information_boundary.py，
直接运行冻结版本的实际感知发布、控制器、原可行性过滤器、新安全层及机器人积分。

矩阵：2 场景 × 3 控制器 × 2 修订执行版本 × 4 观测轨迹 × 3 隐藏状态干预
= 144 对反事实比较。含共同参考，实际为 192 次短时因果回放，每次 30 个积分步。
它们不是新的独立随机性能试验，不能加到正式样本数中。

两场景：V2S2R2、V2S3；三控制器：Ours_rollout、B1、B2。
两版本：strict（仅修复信息边界）、safe（strict 加新安全层）。
四观测轨迹：部分新鲜观测、0.5 s 延迟并带位置噪声、共享失联及轨迹过期、全空观测。
固定 36 只羊和 12 台机器人，其中 8 只羊可有测量轨迹；隐藏个体始终为其余 28 只。
三干预：隐藏个体移至另一远处、移至机器人附近迎面接近、原位置改变运动方向与速度。

每一步固定观测、机器人位置/朝向/速度/角速度及执行层待处理指令。
修改隐藏动物的物理位置和速度后，逐项比较：
1. 实际发布观测（含轨迹时效、来源及覆盖掩码）；
2. 控制器原始输出和模式标签、原生 Rollout 预测结果；
3. FeasibilityLayer 输出；
4. 新 RS 安全层输出；
5. 进入机器人运动学投影前的最终指令；
6. 本步积分后的机器人状态。
比较采用逐元素精确相等，不以小于某个误差阈值代替一致性。

结果：144/144 对通过，最终执行指令的最大分量差为 0。
覆盖中实际调用原生 Rollout 预测 {audit['total_native_rollout_predictions']} 次；
新安全层在 {audit['safe_supervisor_changed_ticks']} 个控制时刻修改了指令，检查并非只走未干预分支。

阳性对照：同一测试改走 archive 的动物真值输入通路，保持观测、机器人状态和
待执行指令相同，改变隐藏真值会改变 FeasibilityLayer 和最终执行指令。
本固定反例的最大指令分量差为
{audit['positive_archive_control']['execution_max_abs_delta']:.6f} m/s。
这是向量指令分量的差，并非机器人实际速度违反上限。
该对照说明本测试能够检出原先的隐藏真值依赖，不只是确认接口字段被删除。

冻结源码及二进制指纹保持为：
{audit['frozen_source_sha256']}

证据范围：这是有限轨迹下的不干扰回归证据，不是对所有状态和未来修改的形式化证明。
固定传感流和隐藏真值干预是因果测试夹具，不应解读为一组自然发生的物理轨迹。
每步开始将机器人状态及下层待处理指令设回共同参考，以隔离信息依赖；
本步实际输出在下一次复位之前已保存并比较。该夹具不夹持过滤器输出。
测试是本次在评估完成后追加的，不能回写为“正式评估前已通过的验收”。
开发预检曾未激活测试障碍物，导致没有调用原生预测；该预检未被作为最终
Rollout 覆盖证据，摘要保留于 analysis/information_boundary_preflight.json。

二、B 阶段：已有观测约束安全控制，但残余接触和性能代价仍存在

新安全层使用可用动物/障碍物观测、速度、轨迹年龄和已知注入噪声标准差。
无动物观测时请求停止；无安全候选时选择预测间隙最大的折中动作并计数。
停止请求仍受有限减速度约束，也不能防止未观测动物主动接近。
rs_no_safe_candidate 只表示有限候选集合中未找到安全动作，不能当作物理不可避免证明。

两场景、30 个新成对种子的 Ours 精确感知对照（共 60 次/版本）：
版本                 机器人—动物接触运行   动物—障碍物接触比例均值   联合完成
archive                    60/60                    2.04%                 54/60
strict                     60/60                    2.04%                 54/60
safe                       15/60                    5.46%                 54/60

因此只能说明新层减少了这组对照中的机器人—动物接触；不能宣称零穿透，
也不能宣称机器人安全改善后原有动物—障碍物安全表现得到无损保留。
全部 1,980 次 safe 运行中仍有 287 次机器人—动物接触；这包含不同控制器和
感知条件的描述性计数，不是 1,980 个完全独立同分布的碰撞概率试验。

三、C 阶段：已计算数据保留，新增完整验收要求尚未全部覆盖

上一轮已完成 1,980 次主感知运行和 360 次分解对照；本次保留全部数据，
没有重跑或扩展矩阵。不能将用户最新提出的所有验收指标说成此前已完成。

已可用：动物—障碍物接触比例、离线真值最小间隙、联合完成、到达、
机器人—动物连续扫掠接触、有限安全候选耗尽计数；所有未完成运行保留。

尚需补齐的统计定义与记录：
- 失检率：必须以符合可探测条件的对象—时刻为分母；现有空观测步数和
  平均观测数量不能直接冒充失检率。
- 实际预警提前量：须按每个威胁事件与消息实际交付时刻定义。旧字段
  ttc_lead 在 1,980 次中有 {status['legacy_ttc_lead_nonfinite_runs']} 次为非有限值，不能据此宣称完整覆盖。
  其中部分可能因无预警事件而未定义；需核对事件分母，不能把这些值直接当作漏检。
- 全闭环计算耗时：原 ctrl_ms 只计 controller.step，不包含感知融合、
  FeasibilityLayer 和新 RS 层；并发批次 wall_s 也不是串行逐周期时延。

信息边界验收通过并不意味着 B、C 的全部应用性论证通过。
现有同一 safe 执行层下的接触比例比较可以报告为限定条件下的补充结果，
但完整的感知质量—失效边界结论必须等待上述指标闭环，不能只凭显著性替代。

四、模型与原论文表述

正式动物环境为 M1，Rollout 使用原生 M0 预测器，这是明确但有限的模型失配。
该配置与本次感知因素分开记录；本次没有改变预测模型来解释感知效应。
原 V7 九场景主表没有被本补充包替换；其原执行层存在真值通路的解释限制仍须说明。
论文 Section 3.3 和 6.2 可统一使用以下限定表述：
“The experiments use the M1 animal dynamics in the simulated environment and
the native M0 rollout predictor. This introduces a defined, limited model
mismatch; it does not establish robustness to arbitrary real-animal behavior.”

本会话工作区未提供独立的 V2_robustness_pilot_results.zip；本说明不否认该归档
存在，也不把用户核对的 44 次先导数据写成此次已独立读取或复算的证据。

五、文件与复现

analysis/verify_information_boundary.py：完整反事实验收脚本。
analysis/information_boundary/summary.json：验收摘要与阳性对照。
analysis/information_boundary/paired_checks.csv：144 对逐项差值。
analysis/information_boundary/replays.json：观测/真值指纹、分支覆盖和轨迹元数据。
analysis/information_boundary/online_traces.npz：各层输出与状态的原始数组。
analysis/acceptance_status.json：三阶段状态及指标覆盖缺口。

在本包根目录，按原 README 编译两个原生库后运行：
  python3 analysis/verify_information_boundary.py
  python3 analysis/build_boundary_addendum.py

核查 NPZ 请使用 numpy.load(..., allow_pickle=False)。本次不需要重跑 1,980 次矩阵。
'''
(ROOT/'Information_Boundary_Acceptance_ZH.txt').write_text(text, encoding='utf-8')
print(json.dumps(status, ensure_ascii=False, indent=2))
