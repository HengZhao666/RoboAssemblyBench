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
| `repro_check_20261124/` | 现成的冒烟启动链（目录名已对应），420 步冒烟曾与原运行逐行相同（`smoke_review.json`） |
| `reference_output_v239/` | 已验证运行的进度行 `progress_history.jsonl`、`idle_hold_trace.jsonl`、`video_frame_clock.jsonl.gz`、退出码与前后源码核验 |
| `compare_progress.py` | 把新运行的进度行与 `reference_output_v239/` 逐行比对 |
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

### 为什么必须用原路径

启动链会在启动时核对身份，并对以下内容做 SHA 断言（见 `launch_round62/checked_worker.py` 开头）：

- worker 配置 `production_worker_config.json`、资产清单 `asset_manifest.json`、900 个源文件；
- 控制器、编译器等 4 个关键模块，要求它们从源码树内加载，按 `resolve()` 后的真实路径判断。

配置和资产清单里写的是绝对路径 `/home/user/beam_integration_20261010_upstream6f57dae/...`，`checked_worker.py`、`tools/*.py`、`fill_worker.py` 里也写死了这个根目录。改这些路径会让 SHA 断言在启动时失败。

所以请在新机器上**按原绝对路径摆放**。`/home/user/beam_integration_20261010_upstream6f57dae` 必须是真实目录或 bind mount，不能是符号链接，否则 `resolve()` 后模块路径不在源码树内，断言失败。这样只需改两个 shell 脚本里的环境值，其余文件一字不改。

### 步骤

1. **环境**：按 [`artifacts/physical_environment/`](../../artifacts/physical_environment/README.md) 建 conda 环境 `internutopia311`、装 Isaac Sim 5.1.0、准备 ffmpeg 9.0.2，并设好 `ISAAC_ASSETS_ROOT`（worker 配置引用 `Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd`）。

2. **摆放文件**（在仓库根目录执行；创建 `/home/user` 下的目录可能需要 sudo）：

   ```bash
   R=/home/user/beam_integration_20261010_upstream6f57dae
   B=$R/grasp_repairs_20261012_20rounds
   A=artifacts/beam_physical
   sudo mkdir -p "$R" && sudo chown "$USER" "$R"
   # 源码树：必须输出 problems: 0
   python3 roboassemblybench/scripts/prepare_physical_reference.py beam --dest "$R/candidate_repository"
   # 启动链（目录名不能改）与实际 worker 配置
   mkdir -p "$B"
   cp -r $A/launch_round62 "$B/round62_v239_unload"
   cp -r $A/repro_check_20261124 "$B/repro_check_20261124"
   gunzip -k "$B/round62_v239_unload/production_worker_config.json.gz"
   cp "$B/round62_v239_unload/production_worker_config.json" "$B/repro_check_20261124/"
   # 基线 worker 与 CAD 几何基准
   mkdir -p "$B/round24_native_release" "$B/batch34_38_20261111"
   cp $A/worker_baseline/full_assembly_worker.py "$B/round24_native_release/"
   cp -r $A/geometry/geometry_tools $A/geometry/joint_geometry "$B/batch34_38_20261111/"
   gunzip "$B/batch34_38_20261111/joint_geometry/scene_geometry.json.gz"
   # python-fcl 与 ffmpeg
   pip install --no-deps --target "$R/runtime_tools/geometry_deps" python-fcl==0.7.0.8
   mkdir -p "$R/runtime_tools/bin" && ln -s "$(conda run -n physical_ffmpeg which ffmpeg)" "$R/runtime_tools/bin/ffmpeg"
   # 核对：worker 配置 SHA 必须是 3bf5d839e72c9d3751d10c265667d377b5efacade3a8ce24b19309c74592f100
   sha256sum "$B/round62_v239_unload/production_worker_config.json" "$B/repro_check_20261124/production_worker_config.json"
   ```

   资产目录中与 Beam 无关的条目是指回本仓库的符号链接，重建后不要移动或删除仓库目录。

3. **只改两个 shell 脚本里的环境值**：`$B/repro_check_20261124/smoke_run.sh`（冒烟）和 `$B/round62_v239_unload/run_full_assembly.sh`（全程）。这两个脚本在运行时不做 SHA 校验，需要改的是：

   | 原值 | 改成 |
   |---|---|
   | `/Embodied_AI/miniconda3/etc/profile.d/conda.sh`、`PATH` 里的 `/Embodied_AI/miniconda3/...` | 本机 conda 安装目录 |
   | `GIT_PYTHON_GIT_EXECUTABLE=/Embodied_AI/miniconda3/bin/git` | 本机 git |
   | `ISAAC_SIM_ROOT=/home/user/isaacsim-5.1.0` | 本机 Isaac Sim 目录 |
   | `CUDA_VISIBLE_DEVICES=GPU-e064bda0-b445-2d7f-50d5-c1180deadea2` | 本机 GPU 的 UUID（`nvidia-smi -L`）或编号 |
   | `VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json` | 本机 NVIDIA Vulkan ICD |

   其余开关和参数都不要改，它们决定 v239 的行为，`checked_worker.py` 也会断言其中一部分。

4. **短冒烟**（420 步，几分钟）：

   ```bash
   bash "$B/repro_check_20261124/smoke_run.sh"
   python3 artifacts/beam_physical/compare_progress.py "$B/repro_check_20261124/smoke_out"
   ```

   预期 `passed: true`：除最后一行（冒烟截止行）外，进度行与已验证运行逐行相同。`smoke_out/` 里的 `runner_config_admission.json`、`runtime_solver_admission.json`、`native_scene_asset_admission.json`、`passive_observer_close.json` 都应为 `passed: true`。重跑前先删掉 `smoke_out/`，脚本用 `test ! -e` 防止覆盖。

5. **全程运行**（约 2 小时；输出约 3.4 GiB；需空闲内存 ≥16 GiB、显存 ≥12 GiB）：

   ```bash
   nohup bash "$B/round62_v239_unload/run_full_assembly.sh" > full_run.log 2>&1 &
   ```

   输出在 `$R/candidate_repository/outputs/fabrica_beam_v239_unload/`；要换位置，改脚本里的 `OUT=`，它不受 SHA 约束。脚本在运行前后都会用 `tools/verify_source.py` 核 900 个源文件。

6. **核对全程结果**：

   ```bash
   python3 artifacts/beam_physical/compare_progress.py "$R/candidate_repository/outputs/fabrica_beam_v239_unload" --full
   ```

   预期全部 353 行进度相同、`exit_code` 为 0。再对照 [`launch_round62/manual_review.json`](../../artifacts/beam_physical/launch_round62/manual_review.json) 的指标：60,524 步、`success-criteria-met`、五件误差、窗口漂移、合力。`reference_output_v239/` 里还有 `idle_hold_trace.jsonl` 和 `video_frame_clock.jsonl.gz`，可逐行比对。

物理设置：步长 1/240 s，TGS 求解器，CPU dynamics。逐行相同只在原 GPU（NVIDIA L20，驱动 580.178.04）上验证过；换硬件后若出现差异，以第 6 步的验收指标为准。

不要用 `launch_round62.py` / `prepare_round62.py`：它们是原批次的带额度启动器，会读写运行记录并拒绝重放。

## 5. 验收口径

- 五件都到目标位姿（3 mm、0.04 rad）。
- 真实释放：两爪全开、无附着、碰撞保持开启；双臂实际回位。
- 五件在共同连续 240 个新物理步内，相对固定参考保持稳定：漂移 ≤2 mm、≤0.03 rad。原始速度单独报告，不作为门槛。
- 最终台面支撑量全部已装件对支撑物的合力：≥9.81 N、≤40 N；底座 6 自身 ≥1.962 N；底面间隙 −0.5…1.0 mm。

禁止用固定连接、写位姿、清速度、关碰撞或重力来制造通过。
