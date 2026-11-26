# 纯物理装配交接：Beam 与 Stool circular（双 UR5e）

整理日期 2026-11-26。两条**纯物理**路线（摩擦抓持、真实释放，不用固定连接、位姿写入、速度清零或关闭碰撞/重力）都已在服务器上完成整件装配。本页是总入口；复现所需的全部代码已收进本仓库，每条路线一份交接。

| 路线 | 结果 | 交接（先读） | 代码与证据 |
|---|---|---|---|
| **Beam**（5 件） | v239，第62轮（2026-11-24）正式运行成功：五件装入、双臂回位、最终无载验证第 240 步通过，60,524 步 | [docs/beam_physical/HANDOFF.md](docs/beam_physical/HANDOFF.md) | [artifacts/beam_physical/](artifacts/beam_physical/) |
| **Stool circular**（9 件） | 正式 R05（P08，2026-11-24）成功；之后去掉暂放台的根治版 P09 全程验证 smoke38 通过，53,191 步 | [docs/stool_physical/HANDOFF.md](docs/stool_physical/HANDOFF.md) | [artifacts/stool_physical/](artifacts/stool_physical/) |

两条路线的授权批次都已按"成功即停"结束。**再跑正式轮须有新的人类指令**；运行状态和授权只以服务器上各自的唯一账本为准（仓库中的 `ledger/` 是 2026-11-26 的快照）。仓库里已有的 [Beam 辅助版](BEAM_REPAIR_README.md) 和 [Stool 辅助版](STOOL_REPAIR_README.md) 是使用虚拟工装的另一条路线，与本页无关。

## 一分钟上手

```bash
git clone https://github.com/HengZhao666/RoboAssemblyBench.git && cd RoboAssemblyBench
# 用 main 中相同的文件 + overlay 重建服务器上实际运行的源码树，并逐文件核 SHA-256
python3 roboassemblybench/scripts/prepare_physical_reference.py beam    # -> outputs/beam_physical_reference/
python3 roboassemblybench/scripts/prepare_physical_reference.py stool   # -> outputs/stool_physical_reference/
```

两条命令都输出 `problems: 0` 才说明重建的树与服务器运行版逐字节相同（2026-11-26 已在 Mac 上实测通过：Beam 904 个文件，Stool 6,475 个文件 + 78 个符号链接）。之后按各路线交接第 4 节运行：最忠实的是在 `pcl-server` 原地复现（源码树、依赖和 GPU 都在）；在新机器上需要 Isaac Sim 5.1.0、conda 环境 `internutopia311`（Python 3.11）和 NVIDIA GPU，并替换启动脚本里写死的服务器路径，这一步尚未在其他机器上验证过。

## 服务器

从项目 Mac 执行 `ssh pcl-server`（实验室 `lab-12`，192.168.251.12，账号 `user`；需校园网或 aTrust）。

| | Beam | Stool |
|---|---|---|
| 源码树 | `/home/user/beam_integration_20261010_upstream6f57dae/candidate_repository` | `/home/user/work/zhaoheng/stool_physical_20261120` |
| 唯一账本 | `…/grasp_repairs_20261012_20rounds/round_ledger.json` | `…/outputs/stool_physical/ledger.json` |
| 最新运行 | `…/grasp_repairs_20261012_20rounds/round62_v239_unload`，输出 `candidate_repository/outputs/fabrica_beam_v239_unload` | `…/outputs/stool_physical/rounds/R05_20261124_104131_P08`、`…/outputs/stool_physical/smoke38` |
| 服务器交接 | `/home/user/beam_integration_20261010_upstream6f57dae/BEAM_PHYSICAL_HANDOFF.md` | 同根 `fabrica_tasks/stool_circular_ur5e/HANDOFF.md` |

## 交接文件完整性检查（2026-11-26）

对服务器与 Mac 工作区做了逐项核对：

- 两条路线的服务器交接与 Mac 工作区副本 SHA-256 相同（检查时及 2026-11-26 同步后均如此；同步前的服务器原件备份在 `grasp_repairs_20261012_20rounds/handoff_sync_20261126/`）（`PROJECT_HANDOFF.md`、`BEAM_PHYSICAL_HANDOFF.md`、Stool `HANDOFF.md`/`CONTRACT.md`、`beam_routes/physical/*.md`、`BEAM_REVIEW_CHECKLIST.md`）。
- 交接中引用的关键路径全部存在：Beam 账本、第62轮启动链/离线检查/输出、复现核对、清理收据、基线 worker、几何基准、`runtime_tools/geometry_deps`；Stool 账本、P08/P09 包、R01–R05、smoke35–38、启动脚本。
- 源码身份：Beam 生产树 900 个文件与第62轮 `source_manifest.json` 逐一相符（0 不符）；Stool 工作树 9 个改动文件与 P09 `source_sha256.json` 逐一相符；P08/P09 补丁 SHA 前缀与交接记载一致（`e8937ecd6f7ceae7`、`25e2c78c6d0ee736`）。
- 账本状态与交接一致：Beam `round62_reviewed_fixed_batch_stopped`（`next_round=63`，无活动轮次）；Stool `R05_verified_success_stopped__root_fix_validated_smoke38`。
- 发现的缺口及处理：
  1. 两条路线都没有可脱离服务器的源码副本（Beam 生产树不是 git 仓库；Stool 基于未推送 GitHub 的 `6f1de1d`，且是不能提交的部分克隆）。**已解决**：以 overlay + SHA 清单 + 重建脚本收入本仓库。
  2. 服务器 Beam 交接"先用五分钟"一段的预期账本状态仍写第58轮，证据表的 `$Q`/`$O` 仍指第58轮（大文件已于 2026-11-24 删除）；本仓库的 Beam 交接以第62轮为准；**已解决**：Mac 工作区主副本与服务器副本均已改正（2026-11-26 同步，SHA 一致）。
  3. 生产树内 `candidate_repository/PROJECT_HANDOFF.md` 的 Stool 行仍写"R05 暂缓"，与服务器根目录的总导航不同步。**已解决**：已用根目录版本同步（2026-11-26）。
  4. Stool 交接中"累计"一行仍停在 smoke26 / 4 轮，"唯一当前状态"一节指向辅助路线账本，容易误读；本仓库的 Stool 交接已单独写明纯物理账本和最新状态；**已解决**：Mac 工作区主副本与服务器副本均已改正（2026-11-26 同步，SHA 一致）。
  5. 本仓库 `main` 上的根目录 `BEAM_PHYSICAL_HANDOFF.md` 还是 2026-10-16（第25轮前）的版本；**已解决**：改为指向新交接，原文副本见 [docs/beam_physical/reference/](docs/beam_physical/reference/)。`beam_routes/physical/` 与 `BEAM_REVIEW_CHECKLIST.md` 同步为最新维护版。
- 未放入仓库：三路视频（只记 SHA，见各路线交接）、大体积 trace/输出、Isaac Sim 与 conda 环境、锁定依赖的 wheel 本体。

## 维护

改动任一路线的代码、配置、验收口径或产生新的正式结果时，在同一次交付中更新对应路线的交接。服务器和 Mac 工作区的交接是日常维护的主副本，本仓库是 2026-11-26 的交付快照。
