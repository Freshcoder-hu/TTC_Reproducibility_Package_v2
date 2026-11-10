## 归档来源及检验

本包由三个原子实验包合并、去重并清理后重建。各原子包的 SHA-256 在打包前逐项校验通过：

- `TTC牧羊_V2_完整可复现包_2026-10-28.zip` → `01_Main_Experiment`，SHA-256 `d9e858b0d0ac7f2e9fa2b752d44b250284a96e38e58ee6aa33287683d23c547f`
- `TTC_Robustness_Experiment.zip` → `02_Perception_Robustness`，SHA-256 `c943bf5adc00c1b03c890c1b11b522eadd22ad77b67b7f149b1ece1268a3f2b9`
- `TTC_Online_Evidence_Supplement.zip` → `03_Online_Warning_and_Timing`，SHA-256 `337df28aa30ed29f026eac2abc630c69276602e68e3a31ab6d7181f20ba430c8`

`v2/stage5/` 模块在重建时从作者归档中找回（15 个文件），作为
`01_Main_Experiment/code/v2/stage5/` 随包提供。

相对路径保持包内相对形式。**不包含论文正文与任何参考文献 PDF**（它们不是实验数据，
且论文另行投稿）。

### 本版相对上一版的改动

1. 删除 40 篇参考文献的 Word 稿（Fig. 1–12）、V7 草稿及其绘图/排版脚本与改写报告；
   论文版本说明改为 `PAPER_VERSION.md`。
2. 找回缺失的 `v2.stage5` 模块，`reproduce_stage5.py` 现可运行（`--check-report` 通过）。
3. 修正全部复现命令；跑不通的命令不再列出。
4. 2,340 格正式语料**只保留一份**（`02_Perception_Robustness/results/formal.jsonl`），
   在线实验目录不再重复存放，改由 `tools/stage_online_inputs.py` 临时挂载。
5. 删除隐藏中间文件 `.formal.jsonl.xIU9Dw`（12.25 MiB）及全部 `__pycache__`。
6. 清除全部绝对 Windows 路径，脚本改用相对包根路径。
7. `LICENSE` 已由占位改为正式双许可：代码 MIT（`LICENSE`），数据与结果
   CC BY 4.0（`LICENSE-DATA`），路径映射见 `LICENSING.md`；同时新增
   `CITATION.cff` 与英文 `README_EN.md`。

### 本次（补入三弯道 / 机器人损毁面板）

8. `05_Curved_Path_240/` 与 `06_Robot_Loss_1080/` 的数据文件**原样复制**自投稿包
   `TTC牧羊_投稿_终版/04_数据/三弯道_240/` 与 `04_数据/机器人损毁_1080/`，逐文件
   SHA-256 与源文件一致（见两个目录的 `README.md` 与
   `PACKAGE_SHA256_MANIFEST.json`），未重算、未改精度、未改 BOM。
9. 投稿用的 9 张图（`Figure_1…9.pdf/.png`）原样复制自
   `TTC牧羊_投稿_终版/03_图表/正文图/`，放在 `04_Quick_Access/Submitted_Figures/`。
10. 删除 V7 时期另一套 9 图排列（`fig01_observability`…`fig09_radar`）的两处副本：
    `01_Main_Experiment/paper/figures/` 与
    `04_Quick_Access/Original_Editable_Figures/Main_V2/`。绘图脚本
    `paper/scripts/build_figures.py` 保留，可按需重建辅助图，但不再随包提供这组图。
11. GitHub 发布准备：新增 `.gitignore`、`.gitattributes`、顶层 `README.md`、
    `00_Documentation/REPOSITORY_URL.md`；`tools/check_hygiene.py` 增加
    GitHub 兼容的 repo 模式；校验清单与发布 zip 在全部修改后重新生成。

`01_Main_Experiment/MANIFEST.json` 是**源归档自带的清单**，其中列出的部分文件
（V7 草稿、Word 稿、上述 V7 图等）是有意不随包提供的；本包的文件清单以
`PACKAGE_FILE_INDEX.csv` 与 `PACKAGE_SHA256_MANIFEST.json` 为准。

