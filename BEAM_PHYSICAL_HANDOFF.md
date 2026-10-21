# Beam 纯物理路线当前交接

更新：2026-10-16，文档整理 W05.0。**暂停；不启动第25轮。** 本次只整理交接/版本/流程，不修改控制源码或恢复实验。完整五件装配尚未成功；交接可直接接续，不代表技术问题已解决。

2026-10-19输出保留范围：用户明确清理C1—C4（v170—200）历史输出副本，C5/v201完整输出、纯物理876源及源码回退封存保留；本轮不改物理代码或恢复实验。当前清理收据见辅助交接的`P/output_cleanup_20261019/cleanup_summary.json`，历史原件链接的可用性以该清单为准。

## 权威、版本与回退

- 服务器 `pcl-server`；R=`/home/user/beam_integration_20261010_upstream6f57dae`；纯物理权威源码 C=`R/candidate_repository`，无Git。旧 `/home/user/RoboAssemblyBench` 仅协调副本。
- 本机 L=`/Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench` 为同步副本。Git树 W=`L/submission/beam_upstream_integration_2026-10-10`，branch=`codex/beam-runtime-repair-20261011`，HEAD=`431050cda779b9a31cae21c548c761705e3b9482`，既存未提交源码必须保留；HEAD不代表完整运行版本，不reset/clean/add-A。
- 冻结候选 **physical-v201/source876**。完整身份：服务器 `R/grasp_repairs_20261012_20rounds/round24_native_release/source_manifest.json`，SHA `2cbd88fac38a1ec6d03ade2933f73383f26edb5d1ec4fdc29e2316edf482877a`；本机同名相对文件在 `outputs/beam_grasp_repairs_20261012/round24_native_release/`。
- 另一个876项平铺清单在 `outputs/beam_assisted_20261015/tools/physical_manifest.json`，SHA `eaa1d2be31830c84aab02feb6d1cc31170c77fb8f9cd41d31e6584815154c91e`。格式不同导致清单自身SHA不同；不能把它误判成源码变化，逐路径/内容核对。
- 原 v200 完整873项回退 SHA `d3228374c1744a0c67f6ed1aea917b7f853db138699954e8d2fe65e2285e297b`；更早首插 source869 回退 `R/grasp_repairs_20261012_20rounds/round15_observer_clock/source_before_observer_clock.tar.gz`，SHA `8bebb60fbce0393e714e7e47b186d586d151854d5c71a02a8d6ba19ccbebbe6f`。恢复前按原清单核真实路径，不只依赖文件名。

## 已到达、失败与未验证

- 最新正式装配尝试 v201/累计24/旧批4 已闭环，26520任务步，`exit0`但`failed=true`、`formal_success=false`。首插及原物理hold96于25854通过，底梁释放于26478通过。
- 严格底梁落桌原速度/支撑共同96仍未通过；既有姿态探索放行不得写成严格落桌成功。
- `hold_3_approach_clearance` 第41步/26519首柱触发原3mm滑移保护，实测4.18364755mm。右高位路径/侧抓、左释放、后续件及最终共同240、多seed尚未执行。
- 直接观察到两处控制缺陷：自动idle-clearance显式目标出现约6.23567rad角度分支偏差；退避26480接用旧技能缓存目标，与实际q最大差1.04851508rad。失败尾段还有跨臂包络重叠候选；末间隔具体接触冲量缺测，不能宣称已隔离唯一动力学根因。

## 下一次获授权后的切入点

先独立审 `demo_policy` 的idle-clearance显式joint覆盖与 `plumbers_block_ur5e_skills` 缓存/失败保持链。以真实释放终点、非零已发布−实测偏差和旧缓存重放：首次开夹、98tick自动分支、退避第一指令；所有回退分支保留实测步幅/跟踪限制。随后重建完整双臂/开夹/持件路径，再考虑一次原生验证。不先调夹力或放宽门。

验收：旧异常反例被拒、合法控制能推进、最终发布与实际路径安全；原前缀回归和释放通过不退化，实际越过退避失败点。阶段改善仍不等于五件成功。本次不实施上述修复。

## 只按需读取

- 本路线检查：[CHECKLIST](beam_routes/physical/CHECKLIST.md)。共用执行：[固定检查](BEAM_REVIEW_CHECKLIST.md)及 `skills/simulation-repair-loop/SKILL.md`。
- 本路线历史：[FINDINGS](beam_routes/physical/FINDINGS.md)，按 `V24`、`T02/T03` 或当前对象/阶段检索，不整篇加载。旧完整交接在 `outputs/beam_assisted_20261015/coordination_history/W04_before_active_handoff_compaction/BEAM_REPAIR_HANDOFF.md`，仅追溯时读。
- 最新原件入口 `outputs/beam_grasp_repairs_20261012/round24_native_release/`：`postreview_conclusions.json`、`release_control_handoff_audit.json`、`video_download_manifest.json`；服务器对应 `R/grasp_repairs_20261012_20rounds/round24_native_release/`，运行输出 `C/outputs/fabrica_beam_v201_native_release_hold`。
- 三完整原视频在同轮 `episode_0000_live_videos/`，每路3465帧/115.5秒，失败窗口约1:48后；原证书、主包及5秒失败尾录保留。
- 最新文档及源身份核验读 `outputs/beam_context_reorganization_20261016/` 中 `source_verification.json`、`sync_verification.json`；若证书不存在/失败即同步待完成，旧实验暂停证书不改写。
