# Beam 辅助路线交接

2026-10-16：D03/A19闭环后暂停，D批3启动/3闭环，D04—D10未授权。路径、源SHA、结果和证据统一读BEAM_REPAIR_HANDOFF.md及P/repair_batch_D_20261016.json。辅助候选v22/source889/tools20保留；原physical876及v21/v20/v19回退保留。原长版见coordination_history/D03_before_pause_compaction/BEAM_ASSISTED_HANDOFF.md。

## 路线选择与允许的辅助

- 原physical与新增assisted都保留；正常选择入口为L/roboassemblybench/scripts/run_beam_route.sh --route physical|assisted --run-name NAME [--max-steps N]。本批只启用assisted，不覆盖运行目录、不公开/更新PR。正式A17实际由P/tools/run_beam_route.sh assisted A17_beam_full 0在已有flock内校验门并claim D01。
- 持件：实测双侧可见CAD表面5—250um、无交叠且12个独立native tick稳定后，在实测位姿建立持件fixed joint；不把辅助固定或0N视觉接触写成纯物理抓稳。原力资格独立保留。
- 安装：实际接近原随机化assembled目标上3mm后建立世界支撑；6夹爪实际全开≤.05rad后，每物理tick锚点≤.1mm/.2mrad缓落。不写Beam位姿/速度、不清速度、不切kinematic、不关闭碰撞，世界支撑不等于自然承重。
- 双臂配合：右臂实际走全开→近夹底梁→视觉保持→最终撤回。右扶维持同run/epoch最近最终发布目标，缺/过期拒绝；双侧可见pad间隙及完整双臂/夹爪/持件每帧采样另验。视觉扶持不冒充实测承力。
- 严格门：插入下降使用真实误差准入（.15mm/1mrad及12独立native tick）并核最终发布q对应持件；模型与native残差原限.1mm/1.5mrad保持。捕获关系累计/瞬态漂移不能由每tick重锚抹去。最终五件全释放、原目标3mm/.04rad、共同240物理步稳定及全序列完成。
- formal_success始终false；仅满足辅助路线完整原生终态才可报告assisted_success。CAD、配置、mock、单阶段、exit0及视频末帧都不替代完整成功。


## v22当前事实

40严格插入固定冷世界轴参考、120um有界横向积分（每tick≤2um、gain.02）、500um实际冷高度越界即停；原严格门、冻结抓取、模型误差与碰撞门不变。12自由阶段保留24原生tick目标交接，缓存按真实身份生命周期。媒体复用捕获统计，H264原帧与编码参数不变。

首柱首次完整安装/松夹/停车，第二柱与右腕实体相交.414831mm后停。三完整视频+7节点已下载SHA同，全部五件与共同240门未过。原扶持报告4保留，同钟诊断2093/2093通过；不冒称承力。

W04.4拒绝把名义平移/Slerp走廊当实际生产轨迹。当前264名义点留8.576mm间隙，但实测失败姿态与最近位置名义点差19.114mm/43.69度；需要生产最终命令+真实入口+误差包络的全持件路径。下一修复方向是左臂先抬高再绕行、右扶保持；当前暂停，不实施D04。

D03_conclusions.json、D03_failed_carry_corridor.json、D03_alignment_native_analysis_reviewed.json、D03_first_column_handoff_analysis.json与D03_timing_summary_reviewed.json为本轮入口。源码/工具889/20前后零差及876核验见D03_final_runtime_state.json；最终暂停同步以证书为准。
