# Beam 纯物理装配 · 复现交接（v239 / 第62轮）

整理日期 2026-11-26。本页面向接手同事：说明最新结果、复现所需的全部代码在仓库中的位置，以及在服务器原地或新机器上的复现步骤。总入口见 [PHYSICAL_ASSEMBLY_HANDOFF.md](../../PHYSICAL_ASSEMBLY_HANDOFF.md)。

## 1. 当前结果

- **生产版本 v239（`v239_unload`），第62轮正式运行成功**：2026-11-24 01:11 启动、03:09 结束，60,524 步、6,875 s（8.80 步/秒），`terminal_reason=success-criteria-met`，超时 0、恢复 0，初始化后禁止的状态写入 0。
- 五件（零件 6、3、2、1、0）全部装入，双 UR5e 真实松爪、回位后进入 `beam_final_unloaded_verification`，第 240 步即通过。五件距目标 ≤0.27 mm、≤0.0042 rad；窗口漂移零件 0 0.109 mm / 0.0012 rad、零件 2 0.029 mm / 0.0006 rad；底座压力 2.39 N，装配总合力 11.93 N（门槛 ≥9.81 N）。
- 全程纯物理：摩擦抓持，没有固定抓取/装配连接，不写物体位姿、不清速度、不关碰撞或重力。
- 账本状态 `round62_reviewed_fixed_batch_stopped`、`active_round=None`、`next_round=63`。按人类指令"授权五轮，或修好停止"，批次 [62, 66] 在第62轮修好后停止，**再跑正式轮须有新的人类指令**。
- 2026-11-24 清理旧输出后做过复现核对（`repro_check_20261124`）：用生产 preflight 重新生成的 worker 配置 SHA 与第62轮相同（`3bf5d839…`），420 步冒烟的 admission 全过，`idle_hold_trace`、`video_frame_clock` 及进度行与第62轮逐行相同。
- 遗留：持件检查时（两手仍夹着）零件 0 对立柱 2 侧向力 4.1 N（修好判据之一要求 <1 N，未满足）；松手后降到 0.26 N，再稳定在约 1.1 N，最终状态与第55轮一致。若要消除，需让零件 0 的插入目标对齐立柱 2 实测 xy（需改控制器，需新指令）。

三路视频（第62轮，服务器 `$O62/episode_0000_live_videos/`，Mac `physical_repair_cc_20261114/v239_unload/round62_videos/`，未放入仓库）：

| 文件 | 字节 | SHA-256 |
|---|---:|---|
| observation_images_front.mp4 | 21,501,123 | `600722cc25ebd35e6c1b62372123c7d17825103e28ffbafc6e9dd9f1dced9427` |
| observation_images_left_wrist.mp4 | 18,199,476 | `d3fead47d767109b69241e97e84e78614304f728ec43548b0d0d3318803a926b` |
| observation_images_right_wrist.mp4 | 13,403,142 | `a1d4e81c74672ffb8d6c880d909a127a0b48111883afd79486d8cd7c4d900241` |

## 2. 仓库里有什么

所有文件在 [`artifacts/beam_physical/`](../../artifacts/beam_physical/)：

| 路径 | 内容 |
|---|---|
| `source_overlay_v239.tar.gz` | 生产树中与本仓库 `main` 不同的 40 个源文件，加 2 个负对照帽件 USD（`beam_contact_mesh_variants/`）。其余 860 个源文件及 Beam 资产与 `main` 逐字节相同，不重复存放 |
| `launch_round62/source_manifest.json` | 第62轮冻结的 900 个源文件清单（相对路径 + SHA-256），复现树按它逐文件核对 |
| `changes_from_main.patch` | 上述 40 个文件中 34 个代码文件相对 `main` 的可读 diff（测试 fixture JSON 只在 tarball 中） |
| `launch_round62/` | 第62轮完整启动链（原样冻结）：`run_full_assembly.sh`（全部环境开关）、`smoke_run.sh`、`checked_worker.py`、`fill_worker.py`、`smoke_entry.py`、`prepare_round62.py`、`launch_round62.py`、`review62.py`、`prefix_check.py`、`tools/`（worker、supervise、preflight、verify_source、CAD guard、资源预算、状态写入审计、流式录像），以及合同/身份/资产清单、预检结果、`manual_review.json`。`production_worker_config.json.gz` 为生产实际 worker 配置（解压后 SHA 以 `3bf5d839` 开头） |
| `offline_v239_unload/` | 离线候选检查：`config_check.json`、`gate_check.json`、`test_config.py`、`run_offline.sh`、`compiler_before62.py`（回退件），`analysis/`（`fixed_criteria.py`、`insert_end.py`、`zstack.py`、`lateral.py`、`unload_rise.py` 及结果） |
| `repro_check_20261124/` | 清理后复现核对：复制的启动链、`smoke_review.json`（与第62轮逐行相同） |
| `cleanup_20261124/` | `setup_repro_check.sh`（生成上面复现核对的脚本）、清理计划与执行记录 |
| `worker_baseline/full_assembly_worker.py` | `checked_worker.py` 复用的基线 worker（服务器原位置 `$B/round24_native_release/`） |
| `geometry/` | CAD 几何基准：`geometry_tools/`、`joint_geometry/`（`scene_geometry.json` 以 gzip 存放，使用前解压到原文件名） |
| `ledger/` | `round_ledger.json`（唯一运行账本快照，2026-11-26）与本批授权 `batch62_66_20261124/` |
| [`roboassemblybench/scripts/prepare_physical_reference.py`](../../roboassemblybench/scripts/prepare_physical_reference.py) | 用 `main` + overlay 重建 900 文件生产树并逐文件核 SHA |

路线历史、机理和逐轮记录：[服务器交接原文（2026-11-24）](reference/BEAM_PHYSICAL_HANDOFF_server_20261124.md)、[纯物理发现 FINDINGS](../../beam_routes/physical/FINDINGS.md)、[纯物理检查 CHECKLIST](../../beam_routes/physical/CHECKLIST.md)、[性能诊断](../../beam_routes/physical/PERFORMANCE_DIAGNOSIS.md)、[通用检查](../../BEAM_REVIEW_CHECKLIST.md)。

## 3. 代码入口

- 任务编排、阶段编译与纯物理判定：`roboassemblybench/core/beam_coordinated.py`、`beam_staged_physical.py`、`beam_physical_compiler.py`、`beam_collision_refinement.py`、`fabrica_canonical.py`。
- 任务状态：`internutopia_extension/tasks/factory_dual_franka_assembly_task.py`。
- 控制器与技能：`toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py`（闭爪位姿门、`_translation_only_recenter_ik` 等）、`beam_support_state.py`（最终支撑合力门）、`beam_physics_diagnostics.py`、`physical_grasp_reference.py`、`demo_policy.py`、`generate_demos.py`。
- Recipe：`fabrica_beam_ur5e_staged`，场景 `taoyuan_grscenes_tabletop`，`--worker-seeds 0 --worker-layout-seeds 505`。
- v231–v239 的行为差异全部由 `run_full_assembly.sh` 中的环境开关选择（不设时与上一版逐字节相同）：

| 开关 | 版本 / 轮次 | 作用 |
|---|---|---|
| `BEAM_CAP0_PICKUP_DEPTH_M=-0.0015` | v231 / 54 | 零件 0 抓取加深，解决提起 `contact_missing` |
| `BEAM_PART0_WINDOW_SPEEDUP=1` | v232 / 55 | 零件 0 下降、搬运、`insert_00` 提速 |
| `BEAM_FAST_MOTION=1`、`BEAM_FINAL_ASSEMBLY_LOAD=1` | v233 / 56 | 第3/2/1件提速；最终台面支撑改量五件对支撑物的合力（人类 2026-11-19 批准） |
| `BEAM_PART0_RELEASE_AFTER_CLEAR=1` | v235 / 58 | 零件 0 松爪只要求到位，五件窗口移到退让阶段 |
| `BEAM_RELEASE_WITHOUT_STACK_WINDOW=1` | v236 / 59 | 去掉退让与最后松爪中的五件 0.5 mm 窗；最终验证不变 |
| `BEAM_HANDOVER_UNLOAD_LIFT=1` | v237 / 60 | 交接合爪后左手把立柱 2 微抬，卸掉其对底座的压力 |
| `BEAM_HANDOVER_UNLOAD_LIFT_M=0.0002` | v239 / 62 | 微抬量 0.2 mm；完成容差 max(抬高/2, 0.4 mm) |
| `BEAM_OPT_*`、`BEAM_*_TRACE_PATH` 等 | 性能/诊断 | 同帧缓存、诊断轨迹；部分观察器会创建接触探针并改变物理，比较轨迹前先读性能诊断 |

## 4. 复现

### A. 在 pcl-server 原地复现（最忠实，已于 2026-11-24 验证到 420 步）

从项目 Mac 执行 `ssh pcl-server`（实验室 `lab-12`，需校园网或 aTrust）。服务器上的生产树仍在，且逐文件 SHA 与清单相符。

```bash
R=/home/user/beam_integration_20261010_upstream6f57dae
C=$R/candidate_repository
B=$R/grasp_repairs_20261012_20rounds
Q62=$B/round62_v239_unload
# 1) 核对生产树没有被改动（900 个文件全部相符才继续）
/usr/bin/python3 - "$Q62/source_manifest.json" "$C" <<'PY'
import hashlib, json, sys
from pathlib import Path
m, root = json.load(open(sys.argv[1])), Path(sys.argv[2])
bad = [f['path'] for f in m['files'] if hashlib.sha256((root/f['path']).read_bytes()).hexdigest() != f['sha256']]
print('mismatch', len(bad), bad[:5])
PY
# 2) 看账本，确认没有正在运行的轮次
/usr/bin/python3 -c "import json;l=json.load(open('$B/round_ledger.json'));print(l['status'],l['active_round'],l['next_round'])"
```

- **不计轮次的冒烟复现**（推荐先做）：照 `$B/cleanup_20261124/setup_repro_check.sh` 的做法，把 `$Q62` 启动链复制到一个**新目录**（脚本里的 `test ! -e "$D"` 防止覆盖），替换目录名，重新跑生产 preflight，核 worker 配置 SHA 与 `$Q62/production_worker_config.json` 相同，再用该目录的 `smoke_run.sh` 跑 420 步，用 `smoke_review.py` 对照第62轮进度行。注意 `smoke_run.sh` 的输出目录必须是新的，旧目录会因 `test ! -e` 拒绝运行。
- **完整运行**（约 2 小时；按第62轮诊断设置输出约 3.4 GiB）：`run_full_assembly.sh` 由 `tools/supervise.py` 调起，`launch_roundNN.py` 会核账本授权、源码/工具/资产 SHA、空闲内存 ≥16 GiB、磁盘 ≥19 GiB、GPU 显存 ≥12 GiB，并把轮次计入账本。**新的正式轮须有新的人类指令**；`launch_round62.py` 不能重放（它断言输出目录不存在、行状态为 `ready_for_native`）。若只为复现而不计轮次，复制启动链到新目录、把 `OUT` 改成新的输出目录后直接 `bash run_full_assembly.sh`，并在交接中注明不计轮次。
- 结束后对照 `$Q62/manual_review.json` 的指标（步数、终止原因、五件误差、窗口漂移、合力）。仿真是确定性的：同一源码、同一 worker 配置下应与第62轮逐行相同。

### B. 在新机器上从本仓库复现（未在其他机器验证）

1. 重建生产树并核 SHA（默认写到 `outputs/beam_physical_reference/`）：

   ```bash
   python3 roboassemblybench/scripts/prepare_physical_reference.py beam
   ```

   输出 `problems: 0` 才表示 900 个源文件和 Beam 资产都与第62轮相同。资产目录中与 Beam 无关的条目以符号链接指回本仓库。
2. 运行环境（与服务器相同的版本）：Isaac Sim 5.1.0（`ISAAC_SIM_ROOT`，并 `source $ISAAC_SIM_ROOT/setup_conda_env.sh`），conda 环境 `internutopia311`（Python 3.11，按仓库 `requirements/`、`environment.yml` 安装），`python-fcl 0.7.0.8`（服务器放在 `$R/runtime_tools/geometry_deps`，加入 `PYTHONPATH`），`ffmpeg`（服务器 `$R/runtime_tools/bin`），NVIDIA GPU 与 Vulkan ICD。`ISAAC_ASSETS_ROOT` 指向 Isaac 官方资产（worker 配置引用 `Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd`）。
3. 几何基准：把 `artifacts/beam_physical/geometry/` 放到 `$B/batch34_38_20261111/` 对应位置，并 `gunzip -k joint_geometry/scene_geometry.json.gz`；把 `worker_baseline/full_assembly_worker.py` 放到 `$B/round24_native_release/`。
4. 启动链：复制 `artifacts/beam_physical/launch_round62/` 为新的轮次目录。冻结脚本写死了服务器路径，需要统一替换：`/home/user/beam_integration_20261010_upstream6f57dae`（`R`，其下 `candidate_repository` 换成第1步的树）、`/home/user/isaacsim-5.1.0`、`/Embodied_AI/miniconda3`（conda 与 git）、GPU UUID `GPU-e064bda0-b445-2d7f-50d5-c1180deadea2`（`CUDA_VISIBLE_DEVICES` 及 `launch_round62.py` 中的 `nvidia-smi --id`）、`/usr/share/vulkan/icd.d/nvidia_icd.json`。`checked_worker.py`、`tools/preflight.py` 会核基线 worker 和工具的 SHA，替换路径时不要改这些文件的内容以外的东西；若必须改，preflight 生成的配置 SHA 会变，需重新记录。
5. 先跑 `smoke_run.sh`（420 步）并用 `smoke_review.py` 对照第62轮，再跑 `run_full_assembly.sh`。物理步 1/240 s、TGS、CPU dynamics；不同 GPU/驱动可能带来数值差异，逐行相同只在服务器上验证过。

## 5. 验收口径（第62轮实际启用）

五件均到目标位姿（3 mm、0.04 rad），真实释放（两爪全开、无附着、碰撞开），双臂实际回位；五件在共同连续 240 个新物理步内相对固定参考保持稳定（漂移 ≤2 mm、≤0.03 rad，人类 2026-11-17 批准的位姿窗口径，原始速度单独报告）；最终台面支撑量全部已装件对支撑物的合力（≥9.81 N、≤40 N），底座 6 自身 ≥1.962 N，底面间隙 −0.5…1.0 mm（人类 2026-11-19 批准）。批准记录在 `launch_round62/human_final_acceptance_approval.json`、`human_instruction_round56.json`。禁止用固定连接、写位姿、清速度、关碰撞或重力制造通过。

## 6. 服务器权威位置

| 内容 | 位置（pcl-server） |
|---|---|
| 生产源码树 | `$R/candidate_repository`（非 git 仓库；以 `source_manifest.json` 核身份） |
| 唯一运行账本 | `$B/round_ledger.json` |
| 第62轮启动链 / 离线 / 输出 | `$B/round62_v239_unload`、`$B/offline_v239_unload`、`$C/outputs/fabrica_beam_v239_unload` |
| 复现核对 / 清理收据 | `$B/repro_check_20261124`、`$B/cleanup_20261124` |
| 服务器交接 | `$R/PROJECT_HANDOFF.md`、`$R/BEAM_PHYSICAL_HANDOFF.md`（与 Mac 工作区同 SHA） |

2026-11-24 已删除第61轮及更早的大文件（trace、视频、PNG，不可恢复），旧轮次只剩小文件；第62轮输出全部保留。
