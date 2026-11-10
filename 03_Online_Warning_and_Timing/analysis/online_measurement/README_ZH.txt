TTC：在线观测、预警与计算时延补充实验包

先读 Online_Evidence_Measurement_ZH.txt。
本包补齐三项指标的测量定义、逐步记录与声明样本内的结果，不改动冻结控制策略。
它是 TTC_Robustness_Experiment.zip 的独立后续补充，原包和原 V7 主表保持原样。

正式性能数据仍为 2,340 次；本次 132 次事件回放、24 次独立串行计时以及
调度预检都是重复测量，不能增加性能推断的样本数。
另有 2 次针对峰值的低分配计时复核，事后单列且未替换原计时面板。

主要文件
  Online_Evidence_Measurement_ZH.txt：中文结果与所有口径限制。
  analysis/online_measurement/protocol.json：测量协议、排程与调度修订说明。
  analysis/online_measurement/detection_summary.csv：可探测分母、交付缺失与轨迹不可用。
  analysis/online_measurement/warning_events.csv：每个潜在遭遇的完整事件记录。
  analysis/online_measurement/timing_summary.csv：独立串行的在线计算时延。
  analysis/online_measurement/phase_timing_by_run.csv：逐阶段时延归因。
  analysis/online_measurement/traces/：事件回放原始数据；并发计时不能用于时延推断。
  analysis/online_measurement/timing_traces/：仅这些串行记录进入时延表。
  analysis/online_measurement/analysis_validation.json：轨迹/指令及分母复核。
  analysis/online_measurement/legacy_warning_audit.json：原 1,328 条非有限 ttc_lead 的定义审计。
  analysis/online_measurement_serial_precheck/：中止的调度预检，未纳入本次样本。
  02_Perception_Robustness/results/formal.jsonl：原 2,340 条正式性能结果的原字节副本（全包只保留这一份，本目录不再重复存放），用于逐项校验。
  code/：与原实验相同的冻结源码与 Linux 原生库。
  MANIFEST.json：本包逐文件 SHA-256。

只阅读成果无需编译。CSV 为 UTF-8，NPZ 可用 numpy.load(..., allow_pickle=False) 读取。
所有时间单位见列名：原始计时为 ns，表格为 ms；仿真事件时间与提前量为 s。

已验证环境：Linux、Python 3.12、NumPy 2.3.5；依赖见 analysis/requirements.txt。
复算统计，在本包根目录运行：
  python3 analysis/analyze_online_evidence.py

完整重复测量（有计算成本，不必为了阅读结果执行）：
  python3 analysis/measure_online_evidence.py
该命令会按既有协议断点续跑，完整记录已存在时不会重复全部运行。
源码、原生库或记录器指纹变化时不能混用原有测量文件，应建立独立版本。
Linux 原生库随包提供；其他系统需自行编译，并建立新的指纹记录。

关键区分
信息已具备预警条件，不等于控制器已经响应；高层响应不等于执行层原样放行。
固定 TTC 外推的参照碰撞时间，不等于实际发生的实体碰撞时间。
无实际接触时不填造碰撞提前量；B1 未实现高层风险预警，不计作 100% 漏警。
这些目标时刻存在时间及机器人相关性，计数不是大量独立伯努利试验。
本次软件计时不证明真实传感器、网络或机器人硬件的最坏实时性。
