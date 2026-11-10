# V2 可复现实验包（TTC 牧羊 · 多机器人 + 动态障碍）

生成：2026-10-28。对应论文 **TTC_Shepherding_CEA_v7**（V2 为唯一实验主体）。

---

## 1. 包内结构

```
code/            仿真源码（41 个顶层 .py + v2/ 49 个模块）
  v2/            V2 五阶段实现（物理执行层、动物 M0/M1/M2、感知与 TTC、几何、统计、可信度检查）
  _rollout.dll   MSVC 编译的滚动预测内核（Windows x64）
  rollout.cpp    该内核的 C++ 源码（可自行重编）
paper/           论文全文（docx + PDF + LaTeX 源 + 9 图 + 表格 CSV + 数字 JSON + 复算脚本）
results/
  results_v2_stage1/      物理执行层验收（17/17）
  results_v2_stage2/      动物模型验收（32/32）+ 图
  results_v2_stage3/      感知与 TTC 标定 + 可观测时间表（26/26）
  results_v2_stage4/      140/220 m 几何 + pilot 筛选
  results_v2_stage5/      正式矩阵 2400 次 + 消融 240 次（另 60 次补充探针）+ 统计
  results_v2_credibility/ 可信度检查（路径偏置 / 穿透取证 / 感知覆盖）
reports/
  V2_STAGE1..5_REPORT.md   五阶段验收报告（含全部判据与数字）
  V2_CREDIBILITY_CHECKS.md  可信度检查报告
MANIFEST.json     447 个文件的路径、字节数与 SHA-256（可用于完整性校验）
```

## 2. 环境

- **Windows x64**，**Python 3.12**（实测 3.12.14 / numpy 2.3.5 / scipy 1.17.0）
- 依赖：`numpy`、`scipy`（`pandas` 仅部分分析脚本用）
- **必须单线程 BLAS**（否则结果不可复现）：

```powershell
set OMP_NUM_THREADS=1
set MKL_NUM_THREADS=1
set OPENBLAS_NUM_THREADS=1
```

## 3. 滚动预测内核

`_rollout.dll` 已随包提供（MSVC 19.44 编译）。如需重建：

```bat
call "<VS>\Common7\Tools\VsDevCmd.bat" -arch=x64
cl /nologo /LD /O2 /EHsc /D_USE_MATH_DEFINES rollout.cpp /link /EXPORT:evaluate_rollouts /OUT:_rollout.dll
```

Linux 上改用 `g++ -O3 -std=c++17 -shared -fPIC rollout.cpp -o _rollout.so`（`proposed_rollout.py` 已按平台选择扩展名）。
`v2/predictor_v2.py` 是同一内核的 NumPy 转写，与 DLL 在 M0 上一致到 2.7e-14。

## 4. 复现全部结果

各阶段验收脚本（在 `code/` 目录下运行）：

```powershell
python -m v2.acceptance_v2          # 阶段1：物理执行层（17/17）
python -m v2.acceptance_s2          # 阶段2：动物模型（32/32）
python -m v2.acceptance_s3          # 阶段3：感知与 TTC（26/26）
python -m v2.acceptance_s4          # 阶段4：几何与 pilot
python -m v2.reproduce_stage5 --check-report   # 阶段5：由 CSV 重算并校验报告逐字节一致
python -m v2.credibility.run_all    # 可信度检查
```

正式矩阵（2400 次，约 3 小时 @jobs=12）：

```powershell
python -m v2.runner_v2 --seeds 2000-2029 --jobs 12 --out results_v2_stage5
```

## 5. 关键配置

| 项 | 值 |
|---|---|
| 配置 id（36:12） | `89fe865b2e1d`（18:6 为 `06897595d088`，60:20 为 `61dc9fae2dff`） |
| 机器人实体半径 | 0.45 m（硬接触距离：机–机 0.90、机–障 1.65、机–羊 0.80 m） |
| 速度/加速度 | $v_{max}$ 4.5 m/s（分层限速：避让时 $v\le\sqrt{a_{lat}r_{req}}$）；$a_{lin}$ 4.0；$a_{lat}$ 2.0 m/s² |
| 机–机安全间距 | 2.0 m（1.2 m 承不住 0.90 m 硬限，实测 36.7 事件/运行） |
| 羊群 : 机器人 | 36 : 12 |
| 动物模型 | **M1**（线性间隙衰减 k=3.0）；M2 仅作压力变体 |
| 感知 | 羊 25 m / 障碍 70 m |
| TTC 阈值 | **$(T_{PRE},T_{DIVERT},T_{REC},h)=(12,8,15,3)$ s**（由实测可观测时间反推；70 m 支撑 12 s，不支持 16 s） |
| 几何 | 140 m（S1/S1N/S2/S2R*/S4*）、220 m（S3），半宽 W=8 m |
| 种子 | 标定 1000–1019 / 评估 2000–2029（互不重叠） |

## 6. 重要说明（与论文一致）

1. **评估种子 2000–2029 已冻结**；标定只用 1000–1019，新增诊断用 1005–1009。
2. **377/2400 次运行在时域内未达终点**（B-PCBF 275、VH-Path 39 等），**全部保留在完成率分母中**。
3. **V2S2 与 V2S4M 几何相同**，论文主表用 **9 场景去重等权**；10 场景口径作为敏感性表。
4. 三条测量口径已修正：预抢占按**控制器信息集**计（旧真值列 300/300 为假，因日志滞后一个积分步、风险带附近差 5.83 s）；$T_{REC}$=15 s 由几何支配（12.5/15/18 逐位相同）；**C1 的零违规不携带信息**。
5. 已知未覆盖项：位置误差 **0.5 m 从未仿真**；延迟 0.5 s 与丢包 10% 仅在解析扫描中；"M2 预测器"是 M1 的代码别名（`predictor_v2.py` 未施加 M2 特征）。
6. 机器–动物交互**未被阻止**（硬检查单步 0.80 m、制动仅覆盖机–机、rollout 无该项）；**不宣称机器–动物无碰撞**。

## 7. 完整性校验

```powershell
python -c "import json,hashlib,pathlib;m=json.load(open('MANIFEST.json',encoding='utf-8'));bad=[f['path'] for f in m['files'] if hashlib.sha256(pathlib.Path(f['path']).read_bytes()).hexdigest()!=f['sha256']];print('OK' if not bad else bad[:5])"
```
