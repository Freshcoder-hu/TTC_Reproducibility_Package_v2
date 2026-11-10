"""Build the Chinese review from completed, paired experiment files."""
import json,gzip,hashlib,os
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from reportlab.pdfgen import canvas
from reportlab.platypus import SimpleDocTemplate,Paragraph,Spacer,Table,TableStyle,PageBreak,Image
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib.enums import TA_CENTER

CODE_ROOT=Path(__file__).parent
ROOT=Path(os.environ.get('TTC_RESULT_ROOT',str(CODE_ROOT))).resolve()
OUT=Path(os.environ.get('TTC_REPORT_OUTPUT',str(ROOT.parent.parent/'output'/'TTC_S3_代码诊断与改进报告.pdf')))
ASSETS=ROOT/'report_assets'
INK='#153D49';TEAL='#127E83';GOLD='#D58B35';GRAY='#EDF3F3'

def rows(folder):
    active=ROOT/'ACTIVE_RESULTS.json'
    if active.exists():folder=json.loads(active.read_text()).get(folder,folder)
    return pd.DataFrame([json.loads(p.read_text()) for p in (ROOT/folder).glob('*__*__*.json')])

def main():
    ref=rows('results_validation_reference');new=rows('results_validation_rollout')
    gen=rows('results_generalization')
    fresh=rows('results_fresh_confirmation')
    assert len(ref)==270 and len(new)==30 and len(gen)==135 and len(fresh)==30,(len(ref),len(new),len(gen),len(fresh))
    frozen=json.loads((ROOT/'MODEL_FREEZE.json').read_text())
    for name,digest in frozen['sha256'].items():
        assert hashlib.sha256((CODE_ROOT/name).read_bytes()).hexdigest()==digest,name
    d=pd.concat([ref,new]);means=d.groupby('variant').mean(numeric_only=True)
    paired=pd.read_csv(ROOT/'analysis_improvement/s3_paired.csv')
    ASSETS.mkdir(exist_ok=True);OUT.parent.mkdir(exist_ok=True)
    font=ASSETS/'DroidSansFallback.ttf'
    if not font.exists():
        import fitz
        font.write_bytes(fitz.Font('cjk').buffer)
    pdfmetrics.registerFont(TTFont('CJK',str(font)))
    pdfmetrics.registerFontFamily('CJK',normal='CJK',bold='CJK',italic='CJK',boldItalic='CJK')
    styles={
      'title':ParagraphStyle('title',fontName='CJK',fontSize=23,leading=32,textColor=colors.HexColor(INK),spaceAfter=12),
      'h':ParagraphStyle('h',fontName='CJK',fontSize=15,leading=22,textColor=colors.HexColor(TEAL),spaceAfter=10),
      'body':ParagraphStyle('body',fontName='CJK',fontSize=9.5,leading=15,wordWrap='CJK',spaceAfter=8),
      'small':ParagraphStyle('small',fontName='CJK',fontSize=8,leading=12,wordWrap='CJK',spaceAfter=6),
      'cell':ParagraphStyle('cell',fontName='CJK',fontSize=8.3,leading=12,wordWrap='CJK'),
      'head':ParagraphStyle('head',fontName='CJK',fontSize=8.3,leading=12,textColor=colors.white,wordWrap='CJK'),
      'code':ParagraphStyle('code',fontName='CJK',fontSize=8,leading=12,wordWrap='CJK',backColor=colors.HexColor(GRAY),borderPadding=8,spaceAfter=10)}
    story=[]
    def p(text,style='body'):story.append(Paragraph(text,styles[style]))
    def table(headers,data,widths=None):
        cells=[[Paragraph(str(v),styles['head']) for v in headers]]
        cells += [[Paragraph(str(v),styles['cell']) for v in row] for row in data]
        t=Table(cells,colWidths=widths,repeatRows=1,hAlign='LEFT')
        t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor(INK)),
          ('VALIGN',(0,0),(-1,-1),'TOP'),('LEFTPADDING',(0,0),(-1,-1),7),
          ('RIGHTPADDING',(0,0),(-1,-1),7),('TOPPADDING',(0,0),(-1,-1),6),
          ('BOTTOMPADDING',(0,0),(-1,-1),6),('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.white,colors.HexColor(GRAY)]),
          ('LINEBELOW',(0,-1),(-1,-1),.5,colors.HexColor('#D1DFDF'))]))
        story.append(t);story.append(Spacer(1,10))
    def page(title):
        if story:story.append(PageBreak())
        p(title,'h')
    pct=lambda v:f'{100*v:.1f}%'
    f3=lambda v:f'{v:.3f}'

    p('S3 实验代码诊断<br/>与改进验证','title')
    p('基于用户上传的完整实验代码；论文参照已修订的英文 v3。<br/>2026-10-20 | 保留原始数据与模型，建立独立修订实验线。','small')
    p('结论：S3 的不利结果同时涉及场景生命周期错误和控制决策缺陷。修正共同环境后，在四状态控制上增加动作条件下的耦合滚动预测，新版本在 30 个独立新种子中降低了接触比例，并改善了平均路径保持。')
    order=['Ours','B2','rollout']
    names={'Ours':'原控制器（共同修正后）','B2':'B2（共同修正后）','rollout':'改进控制器'}
    table(['模型','接触个体比例 ↓','最小距离均值 / m ↑','路径保持率 ↑','联合完成'],[
        [names[k],pct(means.loc[k,'collision_rate']),f3(means.loc[k,'min_dist']),pct(means.loc[k,'rho_avg']),f"{int(round(means.loc[k,'completion']*30))}/30"] for k in order],[160,84,92,82,93])
    b=means.loc['B2'];n=means.loc['rollout']
    q=paired[(paired.reference=='B2')&(paired.metric=='collision_rate')].iloc[0]
    p(f"相对 B2，接触比例减少 {(1-n.collision_rate/b.collision_rate)*100:.1f}%；30 对种子中，{int(q.wins)} 对更低、{int(q.ties)} 对相同、{int(q.losses)} 对更高。该结果支持平均表现改善，不意味着每个种子都优于基线。")
    p(f"种子配对均值差（新 - B2）为 {q.delta:.4f}，配对 bootstrap 95% 区间为 [{q.ci_low:.4f}, {q.ci_high:.4f}]。重采样单位是整次运行，不把 36 只羊视为独立样本。")
    p('验证边界','h')
    p('开发集为 100-104，使用与开发集独立的 200-229 验证 S3，300-304 检查其余九个主场景。初版检查后发现预测截断与恢复阈值的逻辑边界不一致，按规则修正并对最终版本重新复核；代价权重、预测参数及种子未据此调整或筛选。')
    p('其他场景仍存在取舍：单次威胁中，原控制器的部分安全指标更好。具体退步和改善均列在第 4 节，不能写成全场景、全指标占优。')

    page('1  为什么原 S3 会出现逆向结果')
    table(['已确认的问题','代码与运行证据','影响及处理'],[
      ['障碍寿命结束后重复激活','obstacle.py：update() 仅检查 not active 与 t ≥ t_spawn。首障碍在 62 s 结束，62.05 s 又从 (60,18) 出现。','这不符合一次性释放的场景设计。所有方法统一改为只激活一次，避免只给新模型改变环境。'],
      ['当前接触的 TTC 不正确','ttc.py：解析分支没有先检查当前距离。重叠时可能得到负 TTC；重叠但静止或正在分离时可能得到 inf。','共同修正为当前重叠返回 0。这个修正不能消除已发生的接触，也不能单独解释整个 S3。'],
      ['尚未通过就切换威胁','proposed.py：当前 TTC 一次变 inf 就允许更换；解析 TTC 的 horizon 参数未实际截断。','原控制器会过早跟踪二十余秒后的障碍。新版本只在可行动预警域内选候选，并保持当前遭遇直至通过或被紧迫新威胁抢占。'],
      ['威胁、方向和角色不一致','当前威胁带黏性；角色使用全局最小 TTC 对的羊索引与时间；方向受所有活跃障碍的横向位置影响。','保持 TTC、羊索引、障碍对象属于同一对；新事件按其自身来向确定初始方向，重新评价角色。'],
      ['角色跨事件冻结','事件管理重置方向，但不清空角色；旧角色只要仍可行就直接返回。','避让另一侧新威胁时，原执行者可能位置不合适。单独修复并未在开发集稳定改善，最终采用预测评价后的全队一致方向。']
    ],[112,205,194])
    p('第一验证种子 200 的诊断例：原控制器在第二威胁首次达到 12 s 风险阈值之前已开始跟踪它；全程对象更换 23 次。该运行第二、第三障碍分别接触了 10、17 只羊，两者存在个体重合，整次运行接触并集为 25/36。这个例子用于说明机制，不替代 30 种子的总体比较。','small')
    p('另两项统计缺陷：out_risk 在日志 TTC 计算之前读取刚初始化的 inf，因此风险归因不能解释；TTC 汇总仅保留大于 0 的值，会漏掉接触零值。本报告不使用这两项作为改善依据。实际接触比例、距离、路径和完成指标沿用共同评价程序。','small')

    page('2  新版本具体改了什么')
    p('保留 NORMAL、PRE、DIVERT、RECOVER 状态及原预警阈值；先修复事件管理，再增加一个每 1 s 更新的动作选择层。底层机器人控制仍每 0.1 s 更新，实际仿真仍按原 0.05 s 积分。')
    table(['阶段','实际操作'],[
      ['共同修正','corrections.py：所有比较方法采用一次性障碍生命周期及接触 TTC = 0。原代码与旧数据另行保留。'],
      ['事件交接','proposed_event.py：筛除 16 s 之外的候选威胁；当前瞬时 TTC 变 inf 时不立即丢弃尚未通过的事件；方向按当前威胁选择。'],
      ['候选动作','proposed_rollout.py：回中、左右 1.35 / 2.4 / 3.6 m 偏移及保留上次偏移。幅度受当前路径半宽的 0.6 倍约束。'],
      ['耦合预测','rollout.cpp：向前推演 12 s，预测步长 0.15 s；包含机器人槽位、机器人安全过滤、单车运动及羊群 Couzin 相互作用。'],
      ['执行','选中方向由全队一致执行；不再让主要避让组与正常推进组同时给羊群相反的方向。紧接着根据新观测重新规划。']
    ],[98,413])
    p('动作评价函数','h')
    p('J = 100 C + 10 P + 30 B + 0.2 L + 2 S - 0.05 G + 0.02 U','code')
    p('C：预测时域内接触个体比例，同一只羊只计一次。P：羊与障碍的最小间距不足 0.8 m 裕度时的平方惩罚。B：羊体越过实际走廊边界的平均平方惩罚。L：质心横向偏移的归一化平方。S：个体到质心距离超过 12 m 的惩罚。G：沿路径的前进距离。U：本次与上次偏移的变化平方。')
    p('这是一种有限候选的滚动预测控制，不是带可行性证明的完整非线性 MPC。路径和间距进入代价函数，属于软约束；粗步长、部分可观测状态及预测误差仍可能导致接触。')
    p('信息公平性：控制器仅使用当时可用的羊状态、机器人位置、已出现障碍的位置/速度/半径和路径。机器人航向与速度由相邻观测位置估计。未来障碍释放时刻、初始位置、隐藏羊状态和种子编号不进入决策。')
    p('原 role_switches 仍是名义层的诊断计数，不代表新全队执行层的角色切换，不能据此比较新模型的协调负担。','small')
    p('核验：预测积分步长设为原始 0.05 s 时，三种动作的一秒预测统计与独立原 Python 动力学在 1e-8 容差内一致；另有无需隐藏状态/未来障碍字段及恢复边界的运行检查。共 8 项检查通过。','small')

    page('3  S3 全部方法的独立配对验证')
    display=['Ours','Ours_w/o_PRE','B1','B2','VH-Path','VH-Danger','VH-Danger-path','B-Outmost','B-PCBF','rollout']
    table(['方法','接触比例 ↓','最小距离 / m ↑','路径保持率 ↑','联合完成','到达路径目标'],[
        [k,pct(means.loc[k,'collision_rate']),f3(means.loc[k,'min_dist']),pct(means.loc[k,'rho_avg']),
         f"{int(round(means.loc[k,'completion']*30))}/30",f"{int(round(means.loc[k,'success']*30))}/30"] for k in display],
        [116,77,90,80,74,74])
    p('共同条件：同一修正环境、同一观测模型、每个方法同样的 30 个种子（200-229），场景几何及评价函数不按方法改变。原控制器指共同修正后的原 Proposed，并非旧论文表格的归档数值。')
    p('本表不支持“每个单项均最佳”的结论。尤其要结合路径保持与完成判断较低接触值的含义：离开走廊会改变与障碍的暴露。所有方法的原始接触结果均保留。')
    p('“联合完成”沿用原程序的到目标距离与群体分布范围同时达标指标；“到达路径目标”是质心弧长达到路径目标比例。两者回答不同问题。')
    for refname in ['B2','Ours']:
        q=paired[(paired.reference==refname)&(paired.metric=='collision_rate')].iloc[0]
        p(f"与 {refname} 比较：接触比例配对差 {q.delta:.4f}，95% 区间 [{q.ci_low:.4f}, {q.ci_high:.4f}]；新版本 {int(q.wins)} 胜、{int(q.ties)} 平、{int(q.losses)} 负。")
    p('连续指标采用双侧 Wilcoxon，二元指标采用精确 McNemar，并进行 Holm 校正；统计族为 9 个参照方法 × 6 个指标，共 54 项。主要解释依赖配对差和区间，不把“不显著”解释为已证明等效。','small')
    a=new[['seed','collision_rate']]
    fig,axes=plt.subplots(1,2,figsize=(7.2,2.7),dpi=180)
    for ax,k in zip(axes,['Ours','B2']):
        z=a.merge(ref[ref.variant==k][['seed','collision_rate']],on='seed',suffixes=('_new','_ref'))
        ax.scatter(z.collision_rate_ref,z.collision_rate_new,s=22,color=TEAL,alpha=.8)
        ax.plot([0,.85],[0,.85],color='#99A8AD',lw=1,ls='--');ax.set(xlim=(0,.85),ylim=(0,.85),xlabel=f'{k} contacted fraction',ylabel='Revised contacted fraction')
        ax.spines[['top','right']].set_visible(False);ax.grid(alpha=.15)
    fig.tight_layout();figure=ASSETS/'s3_paired.png';fig.savefig(figure);plt.close(fig)
    story.append(Image(str(figure),width=490,height=184))
    p('每个点是一对相同种子的运行，虚线下方表示新版本接触比例较低。','small')

    page('4  其他场景：保留改善，也报告取舍')
    g=gen.groupby(['scenario','variant']).mean(numeric_only=True)
    sids=['S1','S1N','S2','S2R3','S2R2','S2R1','S4S','S4M','S4L']
    table(['场景','原模型接触','B2 接触','新版本接触','新路径保持','新联合完成'],[
        [sid,pct(g.loc[(sid,'Ours'),'collision_rate']),pct(g.loc[(sid,'B2'),'collision_rate']),
         pct(g.loc[(sid,'rollout'),'collision_rate']),pct(g.loc[(sid,'rollout'),'rho_avg']),
         f"{int(round(g.loc[(sid,'rollout'),'completion']*5))}/5"] for sid in sids],
         [64,88,88,88,94,89])
    p('每格均为独立于 S3 开发、验证集的五个检查种子（300-304）。样本量小，用于暴露回归和适用边界，不能替代正式的每场景 30 种子主实验。')
    p('S1 / S1N 无障碍时，新版本与原控制器的接触、路径保持及联合完成结果一致。由于没有障碍，最小障碍距离没有可比意义。')
    p('S2 的具体取舍','h')
    for k in ['Ours','B2','rollout']:
        x=g.loc[('S2',k)]
        p(f"{k}：接触比例 {pct(x.collision_rate)}，最小距离均值 {x.min_dist:.3f} m，路径保持率 {pct(x.rho_avg)}，联合完成 {int(round(x.completion*5))}/5。")
    p('因此，本版本可以作为 S3 的可复核改进方案，但不能声称在 S2 的接触和最小距离上保留了原模型全部优势。后续若专门优化这个取舍，需要使用新的开发集，并重新建立独立验证集；本次没有依据这组检查结果继续调参。')
    p('本模型仍使用理想障碍可见性及原局部羊状态规则。已有的感知实验不能自动移植到新控制器，部署前需要重新运行有噪声、延迟及有限障碍感知距离的实验。','small')

    page('5  消融与恢复边界复核')
    ablation=[]
    for folder,variant,label in [
      ('results_legacy_development','Ours','原环境、原 TTC、原控制器'),
      ('results_environment_only','Ours','仅修正一次性生命周期'),
      ('results_development','Ours','共同修正后的原控制器'),
      ('results_development','handover','再修正角色与风险对一致性'),
      ('results_event_development','event','再修正威胁域与事件交接'),
      ('results_rollout_development','rollout','滚动预测初版（开发选择）')]:
        x=rows(folder);x=x[x.variant==variant].mean(numeric_only=True)
        ablation.append([label,pct(x.collision_rate),pct(x.rho_avg),f"{int(round(x.completion*5))}/5"])
    table(['开发集版本','接触比例 ↓','路径保持率 ↑','联合完成'],ablation,[252,86,91,82])
    p('共同开发种子 100-104。各项修正不保证单调改善：接触 TTC 修正后，该小样本的接触比例反而升高；角色修正也未稳定优于 B2。不能把代码错误存在直接当成其解释全部性能差距的证据。最终版本依据接触、路径及完成表现共同选定，然后冻结。')
    p('初版与最终版本的区别','h')
    p('初版把 16 s 以外的 TTC 截为 inf，但 RECOVER 退出阈值为 17.5 s。这可能把实际为 16.8 s 的风险当成已超过 17.5 s，造成提前退出。最终版保留所有与状态阈值有关的风险值，仅对新事件候选使用 16 s 筛选。')
    p('新增回归检查使用可解析的 16.8 s 迎面遭遇：要求风险值仍是 16.8 s，且 RECOVER 不退出。最终版通过检查，并重新运行 S3 全部 30 个验证种子及九个其他主场景的改进模型。共同修正后的原模型、B2 和外部基线代码不受此修正影响，因此其已完成结果可复用。')
    p('另取 10 个新种子的补充确认','h')
    f=fresh.groupby('variant').mean(numeric_only=True)
    table(['方法','接触比例 ↓','路径保持率 ↑','联合完成'],[
      [k,pct(f.loc[k,'collision_rate']),pct(f.loc[k,'rho_avg']),f"{int(round(f.loc[k,'completion']*10))}/10"]
       for k in ['Ours','B2','rollout']],[160,117,117,117])
    p('最终版冻结后追加种子 500-509，作为恢复边界修正后的补充确认，不用于选模型、调权重或选择报告样本。这里给出描述性结果，不并入主验证的 54 项检验族。')
    p('更密集或更稀疏的障碍释放间隔仍是后续几何泛化检查；本报告不把初版的未完成试运行当作最终版验证证据。','small')

    page('6  如何使用，以及论文需要怎样更新')
    p('文件说明','h')
    p('code/：原动力学与基线、共同修正、事件控制器、滚动预测控制器、C++ 预测核、测试及可续跑驱动。original_code/：用户上传的主程序原样副本。results/：本次开发、独立验证、场景检查、诊断轨迹及模型冻结信息。analysis/：逐种子汇总、配对差、区间和统计表。')
    p('运行环境','h')
    import scipy,platform
    p(f"本次运行：Linux，Python {platform.python_version()}，NumPy {np.__version__}，SciPy {scipy.__version__}，C++17 / g++。Windows 可在 WSL 中复跑。实验只依赖 NumPy、SciPy、pandas 及 g++；报告重建另需 Matplotlib、ReportLab、PyMuPDF。")
    p('进入 code 目录，先运行：<br/>python -m unittest -v test_corrections.py test_rollout.py','code')
    p('正式全场景复跑示例：<br/>export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1<br/>python run_revised_experiments.py --all --seeds 30 --seed-offset 200 \\<br/>--algs Ours Ours_rollout B1 B2 --jobs 8 --out results_revised_full','code')
    p('README 提供完整可复制命令。原 run_experiments.py 保留旧行为；修订实验必须使用新的入口，以确保共同环境修正对所有方法生效。当前计时来自并行验证，不能替代串行实时性基准。')
    p('论文更新要求','h')
    p('本次未直接替换英文 v3 的数值。原 2700 次结果、旧图表与本次共同环境修正后的结果属于不同实验线，不能只把新的 S3 一行粘回旧主表。需要统一重跑主比较，更新方法中的动作预测层、角色执行方式、计算成本、所有相关图表与结论。')
    p('可以支持的表述：在修正共同仿真环境后，面向连续动态威胁的动作条件预测，在独立 S3 配对验证中降低了接触个体比例，并改善了路径保持。仍需明确 S2 的部分安全指标取舍及有限样本的其他场景检查。')
    p('复现差异与保留事项','h')
    p('当前 Linux 环境下原代码 seed 0 的 S3 复跑接触比例为 0.3611，归档表为 0.5833，未复现归档单种子值；未把这种差异直接归因为某个版本或平台。所有新比较均在相同当前环境、相同配对种子下重跑，未把新结果与旧归档值混算。')
    p('机器人仍采用原软安全过滤，未证明机器人与障碍物体不重叠；羊群动力学、理想观测及其他既有假设未借此得到现实验证。完整结果和失败开发版本均保留，便于后续审查。','small')

    def footer(c,doc):
        c.setStrokeColor(colors.HexColor('#C9D8D9'));c.line(42,36,553,36)
        c.setFont('CJK',8);c.setFillColor(colors.HexColor('#55717A'))
        c.drawString(42,23,'TTC 牧羊 | S3 代码诊断与验证 | 2026-10-20')
        c.drawRightString(553,23,str(doc.page))
    doc=SimpleDocTemplate(str(OUT),pagesize=(595.28,841.89),leftMargin=42,rightMargin=42,
                          topMargin=39,bottomMargin=49,title='TTC S3 代码诊断与改进验证')
    doc.build(story,onFirstPage=footer,onLaterPages=footer)
    print(OUT)

if __name__=='__main__':main()
