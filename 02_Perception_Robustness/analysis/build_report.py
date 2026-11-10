"""Build the Chinese supplement from verified complete analysis tables."""
from pathlib import Path
import json,html,io,fitz
import numpy as np
import pandas as pd
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import SimpleDocTemplate,Paragraph,Spacer,Table,TableStyle,Image,PageBreak,KeepTogether
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
ROOT=Path(__file__).resolve().parents[1]
A=ROOT/'analysis';O=ROOT/'output';O.mkdir(exist_ok=True)
S=json.loads((A/'statistics.json').read_text());AU=S['audit'];df=pd.read_csv(A/'all_runs.csv');su=pd.read_csv(A/'summary.csv');pa=pd.read_csv(A/'paired_comparisons.csv');de=pd.read_csv(A/'decomposition.csv')
assert AU['total_runs']==2340 and AU['errors']==0
pdfmetrics.registerFont(TTFont('CJKEmbedded',io.BytesIO(fitz.Font('cjk').buffer)))
INK=colors.HexColor('#193845');TEAL=colors.HexColor('#147d87');PALE=colors.HexColor('#edf5f4');GOLD=colors.HexColor('#b9702f');GRAY=colors.HexColor('#53666e')
styles={
 'kicker':ParagraphStyle('k',fontName='Helvetica',fontSize=9,textColor=TEAL,spaceAfter=18),
 'title':ParagraphStyle('t',fontName='CJKEmbedded',fontSize=26,leading=36,textColor=INK,spaceAfter=18,wordWrap='CJK'),
 'h1':ParagraphStyle('h',fontName='CJKEmbedded',fontSize=18,leading=26,textColor=INK,spaceAfter=12,wordWrap='CJK'),
 'h2':ParagraphStyle('h2',fontName='CJKEmbedded',fontSize=12,leading=19,textColor=TEAL,spaceBefore=11,spaceAfter=5,wordWrap='CJK'),
 'body':ParagraphStyle('b',fontName='CJKEmbedded',fontSize=10,leading=17,textColor=INK,spaceAfter=8,wordWrap='CJK'),
 'small':ParagraphStyle('s',fontName='CJKEmbedded',fontSize=8,leading=12,textColor=GRAY,spaceAfter=6,wordWrap='CJK'),
 'cell':ParagraphStyle('c',fontName='CJKEmbedded',fontSize=8.3,leading=11.5,textColor=INK,wordWrap='CJK'),
 'english':ParagraphStyle('e',fontName='Helvetica',fontSize=9,leading=14,textColor=INK,spaceAfter=9),
}
F={'exact':'精确感知','obs_pos_0p2':'障碍位置 0.2 m','obs_pos_0p5':'障碍位置 0.5 m','animal_pos_0p2':'动物位置 0.2 m','animal_pos_0p5':'动物位置 0.5 m','delay_0p2':'延迟 0.2 s','delay_0p5':'延迟 0.5 s','loss_0p05':'丢包 5%','loss_0p10':'丢包 10%','blackout_0p5':'观测中断 0.5 s','blackout_1p0':'观测中断 1.0 s'}
V={'archive':'原实现','strict':'严格观测','safe':'严格观测+安全层'}
M={'Ours_rollout':'Ours','B1':'B1','B2':'B2'}
story=[]
def P(t,style='body'):return Paragraph(t,styles[style])
def add(t,style='body'):story.append(P(t,style))
def table(rows,widths,pad=5):
 data=[[P(html.escape(str(x)),'cell') for x in row] for row in rows]
 t=Table(data,colWidths=widths,repeatRows=1,hAlign='LEFT')
 t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),PALE),('LINEBELOW',(0,0),(-1,0),.8,TEAL),('LINEBELOW',(0,-1),(-1,-1),.5,TEAL),('VALIGN',(0,0),(-1,-1),'TOP'),('TOPPADDING',(0,0),(-1,-1),pad),('BOTTOMPADDING',(0,0),(-1,-1),pad),('LEFTPADDING',(0,0),(-1,-1),6),('RIGHTPADDING',(0,0),(-1,-1),6),('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.white,colors.HexColor('#f7f9f9')])]))
 story.append(t);story.append(Spacer(1,9))
def page(title):story.append(PageBreak());add(title,'h1')
def footer(c,d):
 c.saveState();w,h=A4;c.setStrokeColor(colors.HexColor('#b9ccce'));c.setLineWidth(.5);c.line(48,39,w-48,39)
 c.setFillColor(GRAY);c.setFont('Helvetica',7);c.drawString(48,27,'TTC SHEPHERDING | OBSERVATION ROBUSTNESS SUPPLEMENT | 2026-10-28');c.drawRightString(w-48,27,str(d.page));c.restoreState()
def r(scene,factor,method,version='safe'):
 g=su[(su.scenario==scene)&(su.factor==factor)&(su.method==method)&(su.version==version)]
 assert len(g)==1;return g.iloc[0]

add('TTC / REPRODUCIBLE EXPERIMENT SUPPLEMENT','kicker')
add('有限感知、观测退化<br/>与机器人-动物安全控制','title')
add('两场景 × 三控制器 × 30 个配对种子；正式计算 2,340 次。','h2')
add('本报告区分三件事：修正闭环的信息边界、改变机器人执行层、检验感知质量下降后的性能。它是 V7 之外的新补充实验，不替代原九场景主表，也不构成真实动物或实体机器人验证。')
siglow=int(((pa.delta_C<0)&(pa.p_holm<.05)).sum());sighigh=int(((pa.delta_C>0)&(pa.p_holm<.05)).sum())
add(f'核心结果：在 44 项预先规定的场景内配对比较中，Ours 的动物-障碍物接触比例有 {siglow} 项显著低于基线、{sighigh} 项显著高于基线，其余 {44-siglow-sighigh} 项未检出差异（Holm 校正）。')
add(f'在新增安全层的 1,980 次运行中，仍有 {AU["safe_rs_contact_runs"]} 次出现机器人-动物接触；另有 1,275 次机器人-障碍物接触、5 次机器人-机器人接触。因此不能宣称零碰撞。记录到的机器人运动学限值违规计数为 {AU["safe_robot_violations"]}。')
add('比较优势仍在，绝对安全性已退化：V2S3 中动物定位每轴标准差 0.5 m 时，Ours 的 C 从精确感知的 0.0972 升至 0.4019；延迟 0.5 s 时升至 0.2231。所测范围内尚未观察到相对 B1/B2 的接触比例优势消失，不能据此确定连续失效阈值。')
add('精确感知下，统一使用新增安全层后的结果','h2')
rows=[['控制器','平均 C','联合完成','机器人-动物接触','路径保持率']]
for m in ['Ours_rollout','B1','B2']:
 g=df[(df.version=='safe')&(df.factor=='exact')&(df.method==m)]
 rows.append([M[m],f'{g.collision_rate.mean():.4f}',f'{int(g.joint_completion.sum())}/60',f'{int(g.any_contact_rs.sum())}/60',f'{g.rho_avg.mean():.4f}'])
table(rows,[67,76,95,135,116])
ours=su[(su.version=='safe')&(su.method=='Ours_rollout')].sort_values('C',ascending=False).iloc[0]
add(f'Ours 在所测条件中接触比例最高的一格为 {ours.scenario} / {F[ours.factor]}：C={ours.C:.4f}，联合完成 {int(ours.joint)}/30，机器人-动物接触 {int(ours.rs_contact_runs)}/30。这个结果只描述已测试档位，不确定连续的失效阈值。')
add('安全改善存在代价：Ours 在精确感知下的机器人-动物接触从原实现的 60/60 降至 15/60，但动物-障碍物平均 C 从 0.0204 增至 0.0546；联合完成均为 54/60。这是两类安全指标的权衡，不能宣称原有性能保持不变。')
add('所有超时、未联合完成和发生接触的运行均保留在对应的 30 次分母中。两场景汇总的置信区间以种子为块重采样，不把同一种子的两个场景当作独立重复。','small')

page('01  实验设计与信息边界')
add('固定条件','h2')
add('场景为 V2S2R2 和 V2S3；36 只动物、12 台机器人；动物/障碍感知半径 25/70 m；积分步长 0.05 s，控制周期 0.1 s。保持 M1 动物环境和 M0 原生 Rollout 预测器，TTC 阈值仍为 (12, 8, 15, 3) s。评估种子为 3000-3029，预检种子为 1100-1101。')
table([['因素','测试档位','范围与解释'],['精确感知','1 档','位置精确、零延迟、无丢包'],['障碍位置误差','0.2 / 0.5 m','每轴独立高斯标准差'],['动物位置误差','0.2 / 0.5 m','每轴独立高斯标准差'],['通信延迟','0.2 / 0.5 s','动物与障碍通道同时延迟'],['独立丢包','5% / 10%','机器人独立发送；保留共享融合'],['短时观测中断','0.5 / 1.0 s','t≥20 s 后每 10 s 发生一次，种子决定相位']], [105,115,269])
add('共 11 条件 × 2 场景 × 3 控制器 × 30 种子 = 1,980 次。另以同一批种子比较原实现、严格观测版和安全层版的精确感知结果；安全层版 180 次已包含在主矩阵，额外增加 360 次。')
add('修正后的闭环','h2')
add('空观测返回空集合；控制器进入停止请求，不恢复全羊群真值。发布给规划器的对象不含全局动物位置、速度或动物模型对象。执行层的可行性过滤与新增安全层均只接收实际发布的动物状态；真实状态仅用于传感仿真、动物动力学、接触监测与评价。')
add('轨迹有效期从名义接收时刻计算，固定为 0.30 s；同时保留源数据年龄。因此 0.5 s 延迟不会被混成“所有消息必然过期”的断联实验。规划器不外推；安全层按可观测速度与时间戳将轨迹传播到当前时刻。')
add('观测中断是全机器人共享的动物通道失联代理；缓存到期后才成为真正空观测。它不是经过视觉渲染的羊群遮挡模型。位置误差为标准差而非最大误差，不能直接与其他论文的绝对定位误差上限作数值等同。','small')

for ix,scene in enumerate(['V2S2R2','V2S3'],start=2):
 page(f'0{ix}  {scene}：感知质量与任务表现')
 story.append(Image(str(A/'figures'/f'{scene}.png'),width=489,height=393))
 rows=[['感知条件','C / Ours','C / B1','C / B2','Ours 联合完成','Ours 机-羊接触']]
 for f in F:
  a,b,c=[r(scene,f,m) for m in ['Ours_rollout','B1','B2']]
  rows.append([F[f],f'{a.C:.4f}',f'{b.C:.4f}',f'{c.C:.4f}',f'{int(a.joint)}/30',f'{int(a.rs_contact_runs)}/30'])
 table(rows,[112,62,62,62,91,100])
 add('C 为原 V7 的动物-障碍接触比例；机-羊接触为连续扫掠监测下发生过接触的运行数。两者属于不同接触类别，不能相互代替。详细间隙、事件数、路径保持和全部完成指标见 summary.csv。','small')

page('04  安全层改变了什么')
rows=[['场景','控制器','版本','平均 C','联合完成','机-羊接触','最差间隙 m']]
for scene in ['V2S2R2','V2S3']:
 for m in ['Ours_rollout','B1','B2']:
  for v in ['archive','strict','safe']:
   a=r(scene,'exact',m,v)
   rows.append([scene,M[m],V[v],f'{a.C:.4f}',f'{int(a.joint)}/30',f'{int(a.rs_contact_runs)}/30',f'{a.rs_clearance_worst:.3f}'])
table(rows,[63,47,105,60,65,74,75],pad=3)
add('机器人自身安全仍不足：Ours 在精确感知的 60 次运行中，机器人-障碍物接触从原实现的 60/60 降至 36/60，机器人-机器人接触从 3/60 降至 0/60。全部新增安全层运行中，机器人-障碍物接触为 1,275/1,980，机器人-机器人接触为 5/1,980。这些是描述性计数，不能把重复条件当作独立试验。')
add('信息边界修正本身在这批精确感知运行中未改变结果：原实现与严格观测版的 180 对运行，C、联合完成、路径保持、机-羊及羊-障最小间隙均逐项相同。这只证明所测两场景三控制器的名义观测情形，不替其他控制器或部分观测条件作保证。')
add('新增安全层在现有可行性过滤之后搜索 25 个速度方向候选，按原机器人限速、转向、加速度与侧向加速度约束预测 1.20 s；同时检查动物、机器人和已观测障碍的预测间距。')
add('动物裕量为 0.15 m + 2×位置噪声标准差 + 0.25×(数据年龄+预测时间)。这是经验裕量；2 倍每轴标准差不是二维误差的 95% 硬界。安全层获知所注入噪声的标准差，属于已知噪声统计条件。')
add('动物定位标准差 0.5 m 时，V2S3 的三个控制器均无机-羊接触，但羊-障接触比例较高。感知输入与按已知标准差扩大的安全裕量共同作用于闭环；本实验未另做固定裕量消融，不能把全部退化归因于测量误差本身。','small')
add('找不到安全候选时，选择预测间距较大的折中动作；这不是不可避免性证明。预测模型仍不包含新增安全层，动物也可能主动接近停止的机器人。因此结果必须按实测接触报告，不能从控制结构推出零碰撞。','small')

page('05  配对统计与可支持的结论')
add('主统计以单场景内的配对种子为单位，比较 Ours 与 B1/B2 的平均 C 差：ΔC = C(Ours) - C(基线)，负值有利于 Ours。使用 10,000 次配对 bootstrap 给出 95% 区间；双侧配对符号翻转检验采用 100,000 次蒙特卡洛抽样，非零差值不超过 17 个时穷举。44 项主比较统一做 Holm 校正。')
add('下表列出精确感知及三个较强扰动档位。完整 44 项比较、未校正 p 值、联合完成的精确 McNemar 检验均保留在 paired_comparisons.csv。联合完成检验作描述性解释，不作等效性或非劣性声明。')
rows=[['场景 / 条件','基线','ΔC','95% 配对区间','Holm p']]
sel=pa[pa.factor.isin(['exact','animal_pos_0p5','delay_0p5','blackout_1p0'])]
for _,a in sel.iterrows():
 rows.append([a.scenario+' / '+F[a.factor],a.baseline,f'{a.delta_C:+.4f}',f'[{a.ci_low:+.4f}, {a.ci_high:+.4f}]',f'{a.p_holm:.4g}'])
table(rows,[174,43,68,137,67])
add(f'校正后显著降低 C：{siglow}/44 项；显著增加 C：{sighigh}/44 项；未检出差异：{44-siglow-sighigh}/44 项。未检出差异不代表两个控制器等效；bootstrap 区间也不是 44 项同时置信区间。')
add('原始运行、错误与实现验证','h2')
add('最终矩阵包含全部 2,340 格，没有运行错误、缺失格或重复格。严格观测相关运行的真值回退计数为 0。此前首轮因旧事后诊断把全局编号用于局部数组而中止；首轮原始记录和异常追踪完整保留，修正后整轮重跑，没有按性能删选运行。')
check=json.loads((A/'validate_mechanisms.json').read_text())
add(f'安全预测器与原运动学在 2,000 组状态上的最大绝对差异为 {check["native_projection_max_abs_error"]:.2e}。另对标量裁剪/二维范数捷径做 60,000 次逐字节比较，并与预检完整运行的主要数值比对一致；这些捷径只减少调用开销。','small')

page('06  论文表述、局限与复现')
add('建议采用的结果边界','h2')
add(f'“在额外的两场景、30 个独立配对种子实验中，我们在统一的严格观测与安全执行层上比较了三种控制器，完成 1,980 次感知条件运行，并以 360 次额外运行分离信息边界修正与执行层修正的影响。全部完成与接触结果均纳入分析。Ours 在 44 项预先规定的场景内接触比例比较中有 {siglow} 项经 Holm 校正后显著较低。新增执行层仍发生机器人-动物接触，因此本研究不声称零穿透或真实动物部署安全。”')
add('不能由本实验推出的结论','h2')
add('1. 本实验不包含真实动物、实体机器人、定位设备数据、视觉识别、真实遮挡或天气。周期性观测中断只模拟共享失联的一种后果。')
add('2. 动物速度、身份和体型仍精确或仅受延迟影响；没有速度估计误差、误检、身份切换或背景杂波。12 台机器人共享观测仍能掩盖独立丢包。')
add('3. 动物模型保留固定路径方向偏置；障碍保持恒速直线运动。M0/M1 的有限失配不能代表未知真实动物行为。')
add('4. 两场景与 36:12 的结果不能推广为九场景、其他规模或实地鲁棒性。所测两档只能给出离散敏感性，不能精确定位失效阈值。')
add('5. 同一批 30 个种子在多条件下重复出现，跨条件的总次数不是同样多的独立统计样本。某格 0/30 接触也不证明接触概率为零；独立二项模型下，0/30 的双侧 95% 精确区间上界约为 11.6%。')
add('与 Liu 等人的比较','h2')
add('Liu 等人的 CMFF+MPC 论文在真实羊群中改变项圈数量，并基于真实定位数据注入噪声和通信延迟；本补充实验增强的是仿真内的信息约束与失效边界证据，仍不填平真实动物验证的差距。该论文的强风试验还同时涉及识别退化与动物应激，不是孤立的传感器误差试验。')
add('来源：H. Liu et al., A robot herding strategy combining CMFF and MPC for shepherding, Computers and Electronics in Agriculture 252 (2026) 112114, Section 3.2.3, pp.14-15；用户上传的 V2 完整实验包及 V7 第 3.3、6.2、6.3 节。','small')
add('复现文件','h2')
add('protocol.json 固定实验口径；results/formal.jsonl 与 manifest 保存每次运行和配置；analysis/all_runs.csv、summary.csv、paired_comparisons.csv、degradation.csv、decomposition.csv 保存全部数值。code/ 包含源码与两个 C++ 内核；README.txt 给出完整编译与运行命令。')
add('代码及二进制指纹（SHA-256）：'+AU['source_sha256'],'small')

replay=A/'failure_replay/summary.json'
if replay.exists():
 page('07  固定种子的密集轨迹复现')
 add('以 V2S3 / Ours / 种子 3009 比较精确感知与 1.0 s 周期性共享观测中断；启用只读状态记录，控制器与参数保持不变。该复现是事后机制检查，不进入新增的统计样本数。')
 rr=json.loads(replay.read_text())
 table([['条件','C','联合完成','机器人-动物最差间隙 m']]+[[F[x['factor']],f"{x['collision_rate']:.4f}",str(bool(x['completion'] and x['success'])),f"{x['min_surface_clearance_rs']:.4f}"] for x in rr],[155,72,85,177])
 story.append(Image(str(A/'failure_replay/replay.png'),width=489,height=289))
 add('精确感知也可能出现接触。失联会改变群体与控制器的整个后续轨迹，最深穿透与动物-障碍接触比例不必沿同一方向变化；单个反例不能替代 30 种子的配对推断。')
 add('这说明新增层只能作为有限候选、有限预测下的安全改善尝试，尚未实现严格防穿透。密集状态 NPZ 与逐步间隙/可观测动物数量 CSV 已一并保留。')

pdf=O/'TTC_Perception_Robustness_Report.pdf'
doc=SimpleDocTemplate(str(pdf),pagesize=A4,rightMargin=48,leftMargin=48,topMargin=48,bottomMargin=53,title='TTC 牧羊：感知鲁棒性与安全执行层补充实验',author='Research supplement')
doc.build(story,onFirstPage=footer,onLaterPages=footer)
print(pdf)
