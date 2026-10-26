# Beam assisted v35：GitHub 草稿交接入口

> **后续个人仓库 main 增补：** 可移植运行入口及实际验证见 [RUN.md](docs/beam_assisted/RUN.md)。下文描述原 v35/A44 交付快照，旧封存包、原报告和原 false 保持不变。新的启动器独立生成当前部署绑定，不使用旧服务器账本；原仓库 PR #11 已撤回，本次不重新提交 PR。

这份提交保存服务器当前辅助路线的完整代码、工具、交接及最新 A44 三路完整录像，供代码审查和后续接手。原仓库比较基线为 `2739e27e66418048d79097376648af1b389a2b68`。根目录保留该基线的其他文件，再覆盖服务器 v35 的889项源文件；服务器原件另以完整快照保存。**这是待审查的草稿，不是已经在最新主线上完成动态验证的合并版。**

## 先读哪些文件

- [完整改动说明](docs/beam_assisted/CHANGES_FROM_UPSTREAM.md)：功能、代码位置、共享默认值变化、19项并行差异及验证边界。
- [服务器原交接](BEAM_ASSISTED_HANDOFF.md)、[辅助检查](beam_routes/assisted/CHECKLIST.md)和[唯一账本原件](outputs/beam_assisted_20261015/repair_batch_D_20261016.json)。账本是暂停状态的历史快照；本PR不提供新运行授权，执行前仍核服务器当前状态及最新人类指令。
- 修复其他任务读 [Fabrica辅助入口](FABRICA_ASSISTED_REPAIR_HANDOFF.md) 和项目技能；不复制Beam数值、结果、计数或权限。

## 完整性与原件位置

- [发布清单](artifacts/beam_assisted/publication_manifest.json)逐文件列出提交内容、SHA256、大小和来源分类。889项源与服务器封存字节一致；130项完整工具位于 `outputs/beam_assisted_20261015/tools/`，其中20项属于A44关键工具封存，不把20误当工具总数。
- [完整服务器代码/工具原字节包](artifacts/beam_assisted/server_code_and_tools.tar.gz)和[包清单](artifacts/beam_assisted/package_manifest.json)保留源码、工具和服务器交接原件。未展开到日常入口的旧同步/历史文件也在包内；Python/test缓存单列排除，外部symlink有明确去向，不伪装成已打包的数据。
- 根目录直接包含所需Beam资产；原仓库已有且与服务器相同的UR5e/Robotiq资产保持不动。详细身份见[资产拉取清单](artifacts/beam_assisted/additional_asset_copy_manifest.json)和[环境/资产审计](docs/beam_assisted/DEPLOYMENT.md)。canonical合同文件仍描述7任务，不能据此声称其他6个任务的外部资产都已随本PR提供。
- [证据索引](artifacts/beam_assisted/evidence_index.json)记录原路径、提交路径、原始及存储SHA。大JSON使用 `.json.gz` 无损保存，原报告内容和false不改；需要读取原相对路径时用下面的恢复命令。
- A44三份完整原视频在 [录像目录](outputs/beam_assisted_20261015/D28_work/A44_media/episode_0000_live_videos/)，每份7445帧、30fps、248.166667秒，未重编码。其他历史日志/视频依用户选择保留服务器；[完整在存文件索引](artifacts/beam_assisted/historical_artifacts_inventory.jsonl.gz)列位置和大小，历史文件内容未在此次重新逐一哈希，既有封存/传输清单仍是其SHA来源。

仅核文件，不加载Isaac、不推进物理：

```bash
python3 scripts/beam_assisted/verify_publication.py
python3 scripts/beam_assisted/verify_publication.py --restore-evidence
```

上述文件校验工具只需Python≥3.7；Isaac运行环境要求另见DEPLOYMENT。第二条仅解压4份索引中的大JSON（约330MB），校验原始SHA，遇到不同的已存在文件会拒绝覆盖；不会启动任务或修改账本。`publication_manifest.json`本身由Git提交身份保护，不自包含哈希。

## 当前结果与后续边界

A44在服务器v35上从头完成五件辅助装配、三段双臂回位和共同240tick，主success/assisted_success/motion_success=true。**独立终态raw仍缺，final_home_review、accepted_full_assembly、all_postreview、formal_success均为false。** 原始报告、原告警及采样缺口保留；辅助连接和安装支撑仍存在，不证明纯摩擦抓稳或撤除支撑后稳定。

源码带有Beam专用资产、路径和数值；部分工具仍按原服务器R/P/A布局、Conda与Isaac环境运行。复现条件在DEPLOYMENT，不把归档快照称作任意机器的一键安装包。服务器同步证书只证明原部署文档；本公开副本为GitHub阅读调整的链接由发布清单单独记录，不能拿旧同步证书认证这些改写。原件仍在代码/工具快照中。

合并前必须处理共享配置差异：服务器夹爪默认kp/kd为7500/173，当前主线为6e6/8e3；fallback pad friction为3/2.5与80/70也不同。后续隔离辅助配置并验证受影响任务，不能仅凭无Git冲突就直接合并。下一修复优先补终止/窗口独立raw取证，不放宽原成功门、不回填历史通过。

自动生成的报告、原件索引和Beam运行资产已标记为GitHub默认折叠，以便先审代码；所有文件仍完整提交，可展开或按发布清单逐项查看。
