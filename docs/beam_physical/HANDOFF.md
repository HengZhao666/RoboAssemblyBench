# Beam 纯物理装配 · 复现说明（v239）

总入口见 [PHYSICAL_ASSEMBLY_HANDOFF.md](../../PHYSICAL_ASSEMBLY_HANDOFF.md)。

## 1. 要复现的结果

v239（`v239_unload`）完整运行：60,524 步，`terminal_reason=success-criteria-met`，超时 0、恢复 0，初始化后禁止的状态写入 0。

- 五件（零件 6、3、2、1、0）全部装入。双 UR5e 真实松爪、回位后进入 `beam_final_unloaded_verification`，第 240 步通过。
- 五件距目标 ≤0.27 mm、≤0.0042 rad。最终窗口漂移：零件 0 为 0.109 mm / 0.0012 rad，零件 2 为 0.029 mm / 0.0006 rad。
- 底座压力 2.39 N，装配总合力 11.93 N（门槛 ≥9.81 N）。
- 已知不足：两手仍夹着时的持件检查中，零件 0 对立柱 2 的侧向力为 4.1 N。松手后降到 0.26 N，之后稳定在约 1.1 N，不影响最终验收。

完整审阅数据在 [`launch_round62/manual_review.json`](../../artifacts/beam_physical/launch_round62/manual_review.json)，其中也记录了三路视频（正面、左腕、右腕）的 SHA-256。

## 2. 仓库里的文件

都在 [`artifacts/beam_physical/`](../../artifacts/beam_physical/) 下：

| 路径 | 内容 |
|---|---|
| `source_overlay_v239.tar.gz` | 与 `main` 不同的 40 个源文件，外加 2 个负对照帽件 USD（`beam_contact_mesh_variants/`）。其余 860 个源文件和 Beam 资产与 `main` 相同 |
| `launch_round62/source_manifest.json` | 运行时的 900 个源文件及 SHA-256 |
| `changes_from_main.patch` | 34 个代码文件相对 `main` 的可读 diff（测试 fixture 只在 tarball 中） |
| `launch_round62/` | 完整启动链：`run_full_assembly.sh`（全部环境开关）、`smoke_run.sh`、`checked_worker.py`、`fill_worker.py`、`smoke_entry.py`、`review62.py`、`prefix_check.py`、`smoke_review.py`、`tools/`（worker、supervise、preflight、verify_source、CAD guard、资源预算、状态写入审计、流式录像），以及合同、身份、资产清单和预检结果。`production_worker_config.json.gz` 是实际使用的 worker 配置，解压后 SHA-256 为 `3bf5d839e72c9d3751d10c265667d377b5efacade3a8ce24b19309c74592f100` |
| `repro_check_20261124/` | 一次复现核对：复制启动链、重新生成 worker 配置（SHA 相同），420 步冒烟与原运行逐行相同（`smoke_review.json`） |
| `cleanup_20261124/setup_repro_check.sh` | 生成上述复现核对目录的脚本，可作为复制启动链的模板 |
| `offline_v239_unload/` | 离线配置检查（`config_check.json`、`gate_check.json`、`test_config.py`、`run_offline.sh`）与分析脚本（`analysis/`） |
| `worker_baseline/full_assembly_worker.py` | `checked_worker.py` 依赖的基线 worker |
| `geometry/` | CAD 几何基准：`geometry_tools/`、`joint_geometry/`（`scene_geometry.json` 以 gzip 存放） |
| `ledger/` | 运行记录 `round_ledger.json` |

物理机理与历史排查见 [纯物理发现 FINDINGS](../../beam_routes/physical/FINDINGS.md) 和 [性能诊断](../../beam_routes/physical/PERFORMANCE_DIAGNOSIS.md)。

## 3. 代码入口

- 任务编排、阶段编译与纯物理判定：`roboassemblybench/core/beam_coordinated.py`、`beam_staged_physical.py`、`beam_physical_compiler.py`、`beam_collision_refinement.py`、`fabrica_canonical.py`
- 任务状态：`internutopia_extension/tasks/factory_dual_franka_assembly_task.py`
- 控制器与技能：`toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py`（闭爪位姿门、`_translation_only_recenter_ik` 等），`beam_support_state.py`（最终支撑合力门），`beam_physics_diagnostics.py`、`physical_grasp_reference.py`、`demo_policy.py`、`generate_demos.py`
- Recipe `fabrica_beam_ur5e_staged`，场景 `taoyuan_grscenes_tabletop`，参数 `--worker-seeds 0 --worker-layout-seeds 505`

v231–v239 的行为由 `run_full_assembly.sh` 里的环境开关选择。不设某个开关时，行为与上一版逐字节相同：

| 开关 | 版本 | 作用 |
|---|---|---|
| `BEAM_CAP0_PICKUP_DEPTH_M=-0.0015` | v231 | 加深零件 0 的抓取，解决提起时 `contact_missing` |
| `BEAM_PART0_WINDOW_SPEEDUP=1` | v232 | 零件 0 的下降、搬运和 `insert_00` 提速 |
| `BEAM_FAST_MOTION=1`、`BEAM_FINAL_ASSEMBLY_LOAD=1` | v233 | 第 3/2/1 件提速；最终台面支撑改为量五件对支撑物的合力 |
| `BEAM_PART0_RELEASE_AFTER_CLEAR=1` | v235 | 零件 0 松爪只要求到位，五件窗口移到退让阶段 |
| `BEAM_RELEASE_WITHOUT_STACK_WINDOW=1` | v236 | 退让和最后松爪中去掉五件 0.5 mm 窗；最终验证不变 |
| `BEAM_HANDOVER_UNLOAD_LIFT=1` | v237 | 交接合爪后，左手把立柱 2 微抬，卸掉它对底座的压力 |
| `BEAM_HANDOVER_UNLOAD_LIFT_M=0.0002` | v239 | 微抬量 0.2 mm；完成容差为 max(抬高/2, 0.4 mm) |
| `BEAM_OPT_*`、`BEAM_*_TRACE_PATH` 等 | 性能/诊断 | 同帧缓存与诊断轨迹。部分观察器会创建接触探针，从而改变物理；比较轨迹前先读性能诊断 |

## 4. 复现步骤

1. **重建源码树并核 SHA**，默认写到 `outputs/beam_physical_reference/`：

   ```bash
   python3 roboassemblybench/scripts/prepare_physical_reference.py beam
   ```

   必须输出 `problems: 0`。资产目录中与 Beam 无关的条目以符号链接指回本仓库。

2. **运行环境**：Isaac Sim 5.1.0、conda 环境 `internutopia311`（Python 3.11）、`python-fcl 0.7.0.8`（加入 `PYTHONPATH`）、`ffmpeg`（加入 `PATH`）、NVIDIA GPU 与 Vulkan ICD。`ISAAC_ASSETS_ROOT` 指向 Isaac 官方资产，worker 配置会引用 `Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd`。

3. **几何基准与基线 worker**：解压 `geometry/joint_geometry/scene_geometry.json.gz`（`gunzip -k`）。启动链按下面的相对位置查找它们，请在一个工作根目录 `R` 下照此摆放：

   ```text
   $R/candidate_repository/                                       第 1 步重建的源码树
   $R/grasp_repairs_20261012_20rounds/<你的运行目录>/              复制自 launch_round62/
   $R/grasp_repairs_20261012_20rounds/round24_native_release/full_assembly_worker.py
   $R/grasp_repairs_20261012_20rounds/batch34_38_20261111/geometry_tools/
   $R/grasp_repairs_20261012_20rounds/batch34_38_20261111/joint_geometry/
   $R/runtime_tools/geometry_deps/                                python-fcl
   $R/runtime_tools/bin/                                          ffmpeg
   ```

4. **改启动链里的固定路径**。复制 `launch_round62/` 为新的运行目录（可参照 `cleanup_20261124/setup_repro_check.sh` 的做法），把脚本中的下列值换成本机的值：

   | 原值 | 含义 |
   |---|---|
   | `/home/user/beam_integration_20261010_upstream6f57dae` | 工作根 `R` |
   | `round62_v239_unload` | 运行目录名 |
   | `/home/user/isaacsim-5.1.0` | `ISAAC_SIM_ROOT` |
   | `/Embodied_AI/miniconda3` | conda 安装目录（含 `bin/git`） |
   | `GPU-e064bda0-b445-2d7f-50d5-c1180deadea2` | GPU UUID（`CUDA_VISIBLE_DEVICES`、`nvidia-smi --id`） |
   | `/usr/share/vulkan/icd.d/nvidia_icd.json` | Vulkan ICD |

   `checked_worker.py` 和 `tools/preflight.py` 会核对基线 worker 与工具的 SHA，除路径外不要改这些文件。改完后重新跑 preflight（见 `setup_repro_check.sh`）。重新生成的 `production_worker_config.json` 中会含本机路径，SHA 不必等于原值；行为一致以第 5 步的逐行对照为准。

5. **短冒烟**：`bash smoke_run.sh`，跑 420 步。输出目录必须不存在（脚本用 `test ! -e` 防止覆盖）。然后用 `smoke_review.py` 与仓库中的进度行对照，预期 `prefix_rows_identical_to_round62: true`。

6. **全程运行**：`bash run_full_assembly.sh`，约 2 小时。按原诊断设置，输出约 3.4 GiB，需要空闲内存 ≥16 GiB、显存 ≥12 GiB。脚本前后会用 `tools/verify_source.py` 核源码。结束后用 `review62.py` 或直接对照 `launch_round62/manual_review.json` 的指标：步数、终止原因、五件误差、窗口漂移、合力。

物理设置：步长 1/240 s，TGS 求解器，CPU dynamics。

## 5. 验收口径

- 五件都到目标位姿（3 mm、0.04 rad）。
- 真实释放：两爪全开、无附着、碰撞保持开启；双臂实际回位。
- 五件在共同连续 240 个新物理步内，相对固定参考保持稳定：漂移 ≤2 mm、≤0.03 rad。原始速度单独报告，不作为门槛。
- 最终台面支撑量全部已装件对支撑物的合力：≥9.81 N、≤40 N；底座 6 自身 ≥1.962 N；底面间隙 −0.5…1.0 mm。

禁止用固定连接、写位姿、清速度、关碰撞或重力来制造通过。
