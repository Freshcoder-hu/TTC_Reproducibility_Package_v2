# 复现指南（中文）

本文件是简短的中文入口。**完整、逐条实测的英文说明见
[`README_EN.md`](README_EN.md)**，论文版本与图表来源见
[`PAPER_VERSION.md`](PAPER_VERSION.md)、
[`FIGURES_TO_EVIDENCE_MAP.md`](FIGURES_TO_EVIDENCE_MAP.md)，
`formal.jsonl` 的 `NaN`/`Infinity` 问题见 [`DATA_NOTES.md`](DATA_NOTES.md)。

包内所有命令均在**本包这一份目录**上实际执行过，运行时间取自本机实测。
跑不通的命令不写进本指南，统一列在 `README_EN.md` 第 7 节。

---

## 一、环境

| 项目 | 值 |
|---|---|
| 验证环境 | Windows 11 x64 |
| Python | **3.13.7**（原始实验在 3.12 上完成） |
| numpy / scipy | **2.5.2 / 1.17.1**（原始为 2.3.5 / 1.17.0） |
| 其他 | pandas 2.3.3（部分分析）、matplotlib 3.10.8（仅画图） |

**必须把 BLAS 固定为单线程**，否则求和顺序变化会改变数字：

```powershell
$env:OMP_NUM_THREADS=1; $env:OPENBLAS_NUM_THREADS=1; $env:MKL_NUM_THREADS=1
```

## 二、实测命令

在包根目录下执行（除注明外）：

| 命令 | 预期输出 | 耗时 |
|---|---|---|
| `python tools/spotcheck.py` | 12 行 `[PASS]`，末行 `RESULT: PASS` | 0.4 s |
| `python tools/standardise_formal.py --verify` | `ROUND-TRIP: PASS …` | 0.7 s |
| `python tools/check_hygiene.py` | 各扩展名文件计数，末行 `HYGIENE: PASS` | 0.4 s |
| `python 01_Main_Experiment/code/reproduce_stage5.py` | `wrote 12 tables + report_numbers.json …` | 1.6 s |
| `python 01_Main_Experiment/code/reproduce_stage5.py --check-report` | `CHECK-REPORT: PASS …` | 1.8 s |
| `python tools/verify_numbers.py` | 先按下方“两类验证”备份 `reproduced/`，再得 `RESULT: PASS - 13/13 files … 0 error(s)` | 0.1 s |
| `python 05_Curved_Path_240/reproduce_table11.py` | 表 11 各行 + `RESULT: PASS - Table 11 / Fig. 8 …` | 0.1 s |
| `python 06_Robot_Loss_1080/reproduce_table12.py` | V2S3 1.67 → 6.94 %、`1080/1080`、`RESULT: PASS …` | 0.2 s |
| `python 01_Main_Experiment/paper/scripts/build_data.py` | 末行 `censored total: 377` / `ablation 300 rows; formal 2400 rows` | 59 s |
| `python 01_Main_Experiment/paper/scripts/build_figures.py` | 9 行 `figure fig0x_…` | 12 s |
| `python tools/stage_online_inputs.py` | `staged hard link …` + 语料 sha256 | 0.2 s |
| `python 03_Online_Warning_and_Timing/analysis/analyze_online_evidence.py` | stdout 输出 JSON，退出码 0 | 5.9 s |
| `python tools/stage_online_inputs.py --clean` | `removed …/results/formal.jsonl` | 0.1 s |
| `python verify_package.py` | `Verified N files; errors = 0` | 见 README_EN §9 |

说明：`build_data.py` / `build_figures.py` 会重写
`results_v2_stage5/reproduced/` 与 `paper/figures/`；同一环境下重复执行结果
字节一致，跨环境则可能只差换行符或浮点最后一位，此时用
`tools/verify_numbers.py` 比较数字（见下）。
`paper/figures/` **不随包提供**：投稿用的 9 张图在
`04_Quick_Access/Submitted_Figures/Figure_1…9`（`03_图表/正文图/` 原样复制）；
`build_figures.py` 只按需重建 V7 时期的辅助排列，其中 `fig07–fig09`
**不是**投稿的图 7–9。
`stage_online_inputs.py` 用硬链接临时挂载共享语料，`--clean` 后目录恢复原状。

## 两类验证（Two kinds of verification）

本包有两道互相独立的验收，不能混为一谈：

1. **压缩包完整性（archive integrity）**——**运行任何生成脚本之前**，每个文件
   都必须与 `PACKAGE_SHA256_MANIFEST.json` 中记录的 SHA-256 完全一致；这正是
   `verify_package.py` 做的事。不一致说明包被改动或损坏，此时任何数值结论都
   无效。
2. **数值复现（numerical reproduction）**——**重新运行生成脚本之后**，比较的是
   **数字**，不是字节。换行符随环境而变（Windows 写 CRLF，POSIX 写 LF），浮点
   最后一位也可能因 NumPy/BLAS/SciPy 版本不同而变化；这些都不改变报告出的
   统计量。`tools/verify_numbers.py` 执行第二类验证。

容差取 `rtol = 1e-12`、`atol = 0`：对量级为 1 的数，它比 IEEE-754 双精度的
舍入误差大约高四个数量级，又远低于本包结果的三位有效小数精度；因此既能吸收
跨环境的最后一位漂移，又不会掩盖真实变化。

Stage-5 用法：

```powershell
# 1. 备份随包提供的输出
Copy-Item 01_Main_Experiment\results\results_v2_stage5\reproduced `
          01_Main_Experiment\results\results_v2_stage5\reproduced_archive -Recurse
# 2. 重新生成
python 01_Main_Experiment\code\reproduce_stage5.py
# 3. 比较数字，而不是字节
python tools\verify_numbers.py
```

工具会把 CRLF/CR 统一为 LF，逐字符比较非数字内容、逐键比较 JSON，并按上述
容差比较每个数字；它列出所有被容忍的差异（仅换行不同的文件各一行 `[EOL]`，
每个被接受的浮点差一行 `[TOL]`），只有缺文件、缺字段/多字段、非数字内容改变
或数字超出容差时才返回非零。不带参数时比较 `.../reproduced_archive` 与
`.../reproduced`，也可用 `--archive` / `--regen` 指定其他目录。备份目录只是
临时验证产物，用完后请删除 `reproduced_archive/`，它不属于随包内容。

## 三、五种子块

| 块 | 种子 | 用途 | 本包是否包含 |
|---|---|---|---|
| 主实验 | **2000–2029** | 8 方法 × 10 场景 × 30 = **2400** 次；消融 R0–R3；机器人损毁面板复用该种子块作配对 | 是（含 `06_Robot_Loss_1080/`） |
| 修正口径 | **3000–3029** | 感知鲁棒性 13 条件 × 3 方法 × 2 场景 = **2340** 格 | 是 |
| 三弯道 | **5000–5029** | 三弯道确认面板，4 个控制行 × 2 场景 × 30 = **240** 次 | **是**（`05_Curved_Path_240/`） |
| 标定 | **1000–1019** | M1/M2 标定、C1 与 T_REC 扫描 | 部分（Stage-5 实为 1000–1004、1000–1002） |
| 诊断 | **1005–1009**、**3000** | 几何/间距诊断；3000 为串行计时种子 | 部分 |

## 四、分母

- 主实验 **2400** 次（含 377 次右删失，全部保留在分母中）；
- 鲁棒性 **2340** 格 = 1980 感知条件 + 360 精确观测对照；
- 在线测量 **132** 次事件回放 + **24** 次独立串行计时；
- 三弯道面板 **240** 次 = 4 个控制行（3 个主方法 + 外部参照 `VH-Path`）× 2 场景 × 30 种子；
- 机器人损毁面板 **1080** 次 = 3 方法 × 3 场景 × 4 条件 × 30 配对种子。

消融 240 次、探针 60 次、132 次回放、24 次计时、**240 次三弯道**与
**1080 次机器人损毁**都是**独立口径**，不得并入上面任何主分母；
`formal_epoch1.jsonl`（1,219 次检查点）已包含在 `formal.jsonl`
内，`formal_attempt1.jsonl` 为旧 attempt，二者均**不可**与正式文件拼接。

## 五、关于 `formal.jsonl`

`02_Perception_Robustness/results/formal.jsonl` 含 `NaN` / `Infinity` /
`-Infinity` 字面量，**不是严格 JSON**（RFC 8259 无这些标记）。此处的
`Infinity` 表示"该接触从未发生"，`NaN` 表示"该阶段未出现"；填入 0 会伪造
t=0 接触与零长度恢复。

包内另附标准化副本 `02_Perception_Robustness/results/formal_standardised.csv`
（4.22 MiB、2340 行、226 列）：非有限值写为空单元格（**绝不写 0**），
真正的 JSON `null` 同样写空但可区分，并在末列 `_nonfinite` 逐项标注，例如
`recovery=NaN;first_contact_time_rr=Infinity`。详见 `DATA_NOTES.md`。

## 六、关于 Stage-5 入口

上一版包中的 `reproduce_stage5.py` 依赖 `v2.stage5`，而该模块当时**不在包内**
（三个实验目录均无），入口直接 `ModuleNotFoundError` 退出（退出码 1）。
本次已从作者归档中**完整找回该模块（15 个文件）**并随包提供，
不是替代脚本。仅改了两处路径：结果根目录指向同级的 `results/`，报告里
打印的 `cd` 一行改为本包内路径。重新生成的 Stage-5 报告与旧版**只差这一行**，
`--check-report` 通过即为证明。

## 七、三弯道与机器人损毁面板

上一版缺的两个实验已按投稿包原样补入，字节未改动：

| 目录 | 内容 | 支撑 |
|---|---|---|
| `05_Curved_Path_240/` | 240 次 = 4 个控制行（`Ours_rollout`、`B1`、`B2` + 外部参照 `VH-Path`）× 2 场景（V2S5A 无障碍对照 / V2S5B 有障碍）× 30 个种子（5000–5029）；另含 seed 5000 的 16 个轨迹 `.npz` | **图 8 / 表 11** |
| `06_Robot_Loss_1080/` | 1080 次 = 3 方法 × 3 场景 × 4 种损毁条件（F0–F3）× 30 个配对种子（2000–2029） | **图 9 / 表 12** |

注意：三弯道是 **4 个控制行**而不是 3 个方法——`VH-Path` 是投稿中标注为
“extension”的外部参照，所以是 4 × 2 × 30 = 240；只算 3 个主方法是 180 次。
三弯道的 `D_turn_centroid_window` 只有 seed 5000 有值（8 行），是单种子示例，
不能当面板均值；表 11 用的是全部 30 个种子都有的字面 `D_turn`。

各自的实测重算入口见第二节，输出与耗时以英文 README 第 3、4.7、4.8 节为准。

## 八、论文版本

本包对应**已投稿**版本：Elsevier *Computers and Electronics in Agriculture*，
LaTeX `cas-dc` 双栏，**9 图、12 表、8 个公式、1 个算法、40 篇参考文献**。
旧版包内的 40 篇参考文献 Word 稿（Fig. 1–12）、V7 草稿及其 9 图排列
（`fig01_observability`…`fig09_radar`，两处副本）均已删除；投稿用的 9 张图
按原样放在 `04_Quick_Access/Submitted_Figures/Figure_1…9`。论文正文另行投稿，
不在本包内。详情见 `PAPER_VERSION.md`。

## 九、适用范围与限制

仅仿真与软件计时；不含真实羊群、实体机器人、相机/网络/硬件实时性结论。
接触度量 `C` 是**一次运行中接触障碍物的动物比例**，不是"每次运行发生碰撞的
概率"；机器人—动物、机器人—障碍物、机器人—机器人接触分开统计。
模型**没有**证明机器人—动物零接触。三弯道面板是**单一构建**，闭环对
10⁻¹² 量级的相对扰动敏感，因此绝对值不可移植（配对比较仍然有效），
其窗口版 `D_turn` 仅 seed 5000 有值。机器人损毁面板考察的是现有控制器在
丢失机器人时的行为：实现中**没有失效检测、没有把失效机器人当障碍物、
也没有失效感知的任务重分配**，因此不能读作“集群可容忍机器人损毁”。
