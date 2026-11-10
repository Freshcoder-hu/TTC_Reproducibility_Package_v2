TTC 牧羊：感知鲁棒性与观测约束安全控制补充实验

请先阅读 Information_Boundary_Acceptance_ZH.txt（2026-10-29 新增验收说明），
再阅读 output/TTC_Perception_Robustness_Report.pdf（此前的 8 页结果报告）。
新增 144 对反事实检查已通过，但这项验收是在性能评估后补做，不能改写时间顺序。
新说明同时列出物理安全代价及失检率、实际预警、全闭环计时的统计缺口。
这是新增补充实验，不修改原 V7 九场景主表；没有真实羊或实体机器人验证。

实验范围
1,980 次感知实验：2 场景 × 3 控制器 × 11 条件 × 30 配对种子。
另有 360 次精确感知对照，用于分离信息边界修正与安全层修正的影响。
最终评估数据为 2,340 个唯一实验格，种子 3000-3029。

文件入口
- protocol.json：事先固定的因素、统计口径、参数与局限。
- analysis/information_boundary/：144 对反事实比较及原执行通路阳性对照的原始证据。
- analysis/acceptance_status.json：三阶段验收状态，区分数据计算完成与全部验收通过。
- results/formal.jsonl：唯一的完整正式原始运行数据。
- results/formal.manifest.json：任务表、配置、软件与源码/二进制指纹。
- analysis/all_runs.csv：全部运行，UTF-8 CSV。
- analysis/summary.csv：每场景、条件、控制器的汇总。
- analysis/paired_comparisons.csv：44 项配对比较、区间、Holm p 值、联合完成对照。
- analysis/degradation.csv：每控制器相对精确感知的接触比例变化。
- analysis/decomposition.csv：原实现、严格观测、新安全层的分解对照。
- analysis/figures/：可独立使用的 PNG/PDF 科研图。
- analysis/failure_replay/：固定反例的密集状态、逐步间隙和复现图。
- source_changes.patch：相对上传源码的修改。
- MANIFEST.json：本包逐文件 SHA-256，可检查完整性。

重要口径
C 是每运行中曾接触障碍物的动物比例，不能当作“运行发生接触的概率”。
机器人-动物、机器人-障碍物、机器人-机器人接触另行统计。
观测中断是周期性共享失联代理，不是真实视觉遮挡。
位置噪声为每轴高斯标准差；速度、身份、体型仍精确或只受延迟影响。
安全层获知注入噪声的标准差；它按预定规则放大裕量。
本实验没有单独消融固定裕量，因此测得的是该完整闭环对误差的反应。
新增安全层未实现零穿透，机器人与障碍物接触仍多。
两场景新种子复跑数值不能与原 V7 九场景平均值混用。

避免重复计数
formal_epoch1.jsonl 是中断时的 1,219 次检查点，已经包含在 formal.jsonl 中。
formal_attempt1.jsonl 是旧事后诊断越界前的中止批次，整轮已按修正版本重跑。
pilot_v1、regression_blackout 与 failure_replay 是预检或重复验证，不能增加正式 N。
请只使用 results/formal.jsonl 或 analysis/all_runs.csv 作为正式评估数据。
所有超时和未联合完成运行均保留在分母中。

复跑（实际验证环境为 Linux / Python 3.12）
依赖版本见 analysis/requirements.txt；完整英文说明见 README.txt。
在 code 目录编译：
  g++ -O3 -std=c++17 -shared -fPIC rollout.cpp -o _rollout.so
  g++ -O3 -ffp-contract=off -std=c++17 -shared -fPIC rs_safety.cpp -o _rs_safety.so
回到本包根目录：
  export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
  python3 analysis/validate_mechanisms.py
  python3 analysis/verify_information_boundary.py
  python3 analysis/build_boundary_addendum.py
  python3 code/run_robustness.py --out results/reproduced.jsonl --include-decomposition --jobs 9
默认即为完整 2,340 次矩阵。新结果请使用新的输出文件名。
如需中断续跑，复用同一输出文件名；源码与二进制指纹不一致时程序拒绝续接。
分析器默认读取 formal.jsonl；在一份新的实验包副本中，将复跑数据及对应 manifest
置于该路径，再运行 analysis/analyze.py 和 analysis/build_report.py。
请保留本包原始正式数据，不直接覆盖唯一副本。
