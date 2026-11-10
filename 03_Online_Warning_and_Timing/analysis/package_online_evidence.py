"""Package a standalone measured-evidence supplement; keep prior ZIP intact."""
from pathlib import Path
import hashlib
import json
import zipfile

ROOT=Path(__file__).resolve().parents[1]
MEASURE=ROOT/'analysis/online_measurement'
status=json.loads((MEASURE/'summary.json').read_text())
assert status['status']=='complete' and status['sample_runs']==132 and status['serial_timing_runs']==24
dataset=ROOT/'results/formal.jsonl'
assert hashlib.sha256(dataset.read_bytes()).hexdigest()=='d8523bd9e2349cc68576454429ffd3638c4b5f1f918649b2fcee5ef8d062496d'

closure=dict(status='measurement_evidence_complete_within_declared_sample',
    primary_performance_N=2340,measurement_event_replays=132,independent_serial_timing_replays=24,
    targeted_low_allocation_timing_replays=2,
    controller_or_execution_changes=False,source_hash_unchanged=True,
    evidence='Online_Evidence_Measurement_ZH.txt',
    limits=['Two seeds per event cell; one seed per serial timing cell',
            'Latency measured only for exact, animal_pos_0p5, delay_0p5, blackout_1p0',
            'No hardware real-time or real-animal guarantee',
            'Residual robot-animal contacts and animal-obstacle safety tradeoffs remain'],
    previous_acceptance_status='The earlier acceptance_status.json describes the pre-measurement stage and is not rewritten retrospectively')
(MEASURE/'closure_status.json').write_text(json.dumps(closure,indent=2))

readme='''TTC：在线观测、预警与计算时延补充实验包

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
  results/formal.jsonl：原 2,340 条正式性能结果的原字节副本，用于逐项校验。
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
'''
(MEASURE/'README_ZH.txt').write_text(readme,encoding='utf-8')

items={'README_ZH.txt':readme.encode()}
for relative in ('Online_Evidence_Measurement_ZH.txt','protocol.json','source_changes.patch',
                 'results/formal.jsonl','results/formal.manifest.json',
                 'analysis/requirements.txt','analysis/measure_online_evidence.py',
                 'analysis/analyze_online_evidence.py','analysis/package_online_evidence.py'):
    p=ROOT/relative
    if p.exists():items[relative]=p.read_bytes()
items['analysis/plot_online_evidence.py']=(ROOT/'analysis/plot_online_evidence.py').read_bytes()
items['analysis/check_timing_overhead.py']=(ROOT/'analysis/check_timing_overhead.py').read_bytes()
for directory in ('code','analysis/online_measurement','analysis/online_measurement_serial_precheck'):
    for p in sorted((ROOT/directory).rglob('*')):
        if p.is_file() and '__pycache__' not in p.parts:
            items[str(p.relative_to(ROOT))]=p.read_bytes()
manifest=dict(closure=closure,source_sha256=json.loads((MEASURE/'execution_status.json').read_text())['frozen_source_sha256'],
    formal_data_sha256=hashlib.sha256(dataset.read_bytes()).hexdigest(),
    files=[dict(path=k,size=len(v),sha256=hashlib.sha256(v).hexdigest()) for k,v in sorted(items.items())])
items['MANIFEST.json']=json.dumps(manifest,ensure_ascii=False,indent=2).encode()
output=ROOT.parent/'output/TTC_Online_Evidence_Supplement.zip'
with zipfile.ZipFile(output,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for name,data in sorted(items.items()):z.writestr('TTC_Online_Evidence_Supplement/'+name,data)
with zipfile.ZipFile(output) as z:
    assert z.testzip() is None
    for f in manifest['files']:
        b=z.read('TTC_Online_Evidence_Supplement/'+f['path'])
        assert hashlib.sha256(b).hexdigest()==f['sha256']
print(json.dumps(dict(path=str(output),bytes=output.stat().st_size,manifest_files=len(manifest['files']),crc='pass')))
