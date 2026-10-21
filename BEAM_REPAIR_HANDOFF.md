# Beam 短总入口

通用流程 W06.0，2026-10-21。一轮合并可分别验证的已定位无冲突问题，按差异重验、复用未变部分；具体规则见共用检查。此入口只导航，不提供仿真授权；当前状态和权限只读所选路线交接。默认简短解释故障位置和可能根因，不另写报告。必要原件、机器校验和短交接仍保留。

修复本项目其他Fabrica辅助任务，改读[新任务辅助修复入口](FABRICA_ASSISTED_REPAIR_HANDOFF.md)；它不继承Beam的运行授权、结果和参数，也不默认加载本路线历史。

## 选一路，只读一路

- 用户指定physical/无辅助/纯物理：读 [纯物理当前交接](BEAM_PHYSICAL_HANDOFF.md)，再读 `beam_routes/physical/CHECKLIST.md`。
- 用户指定assisted/物理辅助：读 [辅助当前交接](BEAM_ASSISTED_HANDOFF.md)，再读 `beam_routes/assisted/CHECKLIST.md`。
- 沿用当前辅助修复时只读辅助入口；未明确选择且会影响操作时先澄清。源码C对应physical，源码A对应assisted；共享根及本机外层不能仅凭目录推断路线。
- 不自动读取另一条路线、整个发现索引、混合历史和旧会话。需要旧证据时仅在所选路线索引按当前瓶颈检索；历史状态不覆盖当前交接。

## 共用方法与版本

共用 [固定检查](BEAM_REVIEW_CHECKLIST.md)；需要执行/复盘时用 [simulation-repair-loop](skills/simulation-repair-loop/SKILL.md)，详细控制/复用和流程优化参考按需加载。[流程记录](BEAM_WORKFLOW_LOG.md)只记录方法变更，不承载实验授权；[发现入口](BEAM_FINDINGS_INDEX.md)只做路线导航。

控制源码版本、关键工具版本、运行编号、文档版本和技能版本独立。每条路线自己维护候选/回退清单SHA和运行账本；共享本机Git HEAD不等于完整运行身份。不得把另一条路线结果或计数纳入当前路线。

机器可读路径/身份及当前同步证书见 `beam_routes/registry.json`；文档清单见 `beam_routes/docs_manifest.json`。入口整理原件留在 `outputs/beam_context_reorganization_20261016/`，后续整理前原件见所选路线交接；当前同步证书不存在或失败时为sync_pending。服务器源码权威，逐项核验后才能称一致；旧实验终结证书不覆盖、不重写。
