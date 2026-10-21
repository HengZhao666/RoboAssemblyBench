# Beam 发现索引：只读所选路线

当前状态、授权和候选以路线当前交接为准；发现索引保存历史机制、反例、否定路线及原件指针，不整篇默认加载。

- physical：[当前交接](BEAM_PHYSICAL_HANDOFF.md)；[本路线发现](beam_routes/physical/FINDINGS.md)。在本路线按阶段/对象/版本检索。
- assisted：[当前交接](BEAM_ASSISTED_HANDOFF.md)；[本路线发现](beam_routes/assisted/FINDINGS.md)。在本路线按阶段/对象/版本检索。

共用的审查、证据复用和流程优化方法只维护在 `skills/simulation-repair-loop/`，不把路线的物理机制、阈值或实验数值写进去。整理前原文完整保留在 `outputs/beam_context_reorganization_20261016/before/BEAM_FINDINGS_INDEX.md`，只有追溯原文时读取。
