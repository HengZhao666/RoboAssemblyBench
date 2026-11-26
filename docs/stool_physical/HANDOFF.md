# Stool circular 纯物理装配 · 复现说明（P09）

总入口见 [PHYSICAL_ASSEMBLY_HANDOFF.md](../../PHYSICAL_ASSEMBLY_HANDOFF.md)。本路线与仓库里已有的 [Stool 辅助装配版](../../STOOL_REPAIR_README.md)（使用虚拟工装）是两条独立路线，代码、配方和验收都不同。

## 1. 要复现的结果

**推荐复现 P09**，全程运行结果：

- 九件全部纯物理装入，`success-criteria-met`，53,191 步，零超时、零恢复。
- 双臂空手回位，最终无载静置 30/30 步通过。
- 验收误差最大 1.48 mm / 0.039 rad；相对底板原名义装配位 ≤1.59 mm / 0.043 rad。
- 13 次抓取全部为 `pure_physical_grasp`：无固定关节、无位姿写入或锁定，抓取时保持碰撞。

上一版 P08 也通过完整验收（53,580 步，逐件误差见 [`runs/R05_20261124_104131_P08/review.json`](../../artifacts/stool_physical/runs/R05_20261124_104131_P08/review.json)，其中也记录了三路视频的 SHA-256）。P08 在板上加了一块 108×108×3 mm 的被动静态暂放台供帽件换抓；P09 去掉了它，改为光学板碰撞用同顶面长方体、板摩擦 0.8/0.6。

相对原 Stool 任务合同，以下改动需要知道：

- 帽件的成功目标在取件前按所插腿的实际位姿重定位，容差不变。
- 帽件两指严格接触改为用 PhysX 实测接触点配对，并计入 1 mm SDF 余量。
- 帽件采用"边夹—放直—深 10 mm 重抓"的换抓流程。
- 腿 1、帽 0、帽 8 限开度松爪。
- 光学板碰撞改为长方体：顶面不变，孔被填平（Stool 不用孔）。

## 2. 仓库里的文件

都在 [`artifacts/stool_physical/`](../../artifacts/stool_physical/) 下：

| 路径 | 内容 |
|---|---|
| `source_overlay_P09.tar.gz` | 运行时源码树（Plumbers 集成框架提交 `6f1de1d` 加 P09 改动）中与 `main` 不同的 4,008 个文件和 78 个符号链接，主要是集成框架 `integration/`、冻结兼容树 `compat/`、Plumbers 物理与 Stool 配方。其余 2,467 个文件（含用到的全部资产）与 `main` 相同 |
| `source_manifest.json` | 全部 6,475 个文件的 SHA-256 和来源（`main` / `overlay`），外加 78 个符号链接 |
| `changes_P09_vs_6f1de1d.patch` | Stool 纯物理相对 Plumbers 框架的全部改动：9 个文件，可读 diff |
| `packages/P01…P09` | 各版本的 `delta_from_6f1de1d.patch` 与源文件 SHA |
| `run_tools/` | `run_smoke.sh`（全程或调试运行）、`launch_round.py` + `start_round.sh`（带资源检查的正式运行）、`watch.sh`（按阶段跟日志）、`py.sh`（带运行环境的离线 Python），以及离线检查脚本（`*_probe.py`、`*chk.py`）和补丁脚本（`patch01…13.py`） |
| `runs/` | 各次运行的 `review.json`、`launch.json`、`launcher.log`、`native/validation_summary.json`；`runs/smoke38/` 是 P09 的参考结果 |
| `ledger/ledger.json` | 运行记录与验收定义 |

## 3. 代码入口（P09 改动的 9 个文件）

所有改动都不改变 Plumbers 的原有行为，Stool 通过配方开关启用：

- **配方** `roboassemblybench/tasks/fabrica_stool_circular_ur5e_physical/recipe.yaml`、`annotation.yaml`：`fabrica_stool_circular_ur5e_physical`，继承 `fabrica_stool_circular_ur5e_staged`。包含左右手指垫刚度和阻尼、臂 PD（kp 640000 / kd 11313.7），以及开关 `physical_seat_before_release`、`physical_rebase_targets_to_socket`、`physical_cap_regrasp`、`physical_servo_reference_point_parts`、`physical_limited_release`、`physical_board_collision: boundingCube`、`physical_board_friction`。
- **编译逻辑** `roboassemblybench/core/plumbers_physical.py`：接触盒对齐抓取、座入后再释放、帽件重定位、换抓阶段、限开度松爪、实测接触点严格配对。
- `roboassemblybench/core/fabrica_canonical.py`：连续物理分支扩展到 `stool_circular`（`_GENERIC_PHYSICAL_ASSEMBLIES`），以及光学板碰撞近似与摩擦。
- `roboassemblybench/core/plumbers_support.py`：`add_terminal_joint_home` 只在存在支撑撤离引用时改写最后一次撤离。
- `internutopia_extension/tasks/factory_dual_franka_assembly_task.py`：可选的零件系 `contact_box_orientation`、`keep_anchor_target`，以及实测接触点配对用的 `strict_force_point_surface_margin`。
- `toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py`：伺服和收敛参考点 `target_object_reference_point`，闭爪 trace 附接触详情。
- `toolkits/factory_dual_franka_assembly/demo_policy.py`：空闲臂保持允许 Stool。
- `integration/phase_journal.py`：按任务实际零件名记录。

## 4. 复现步骤

Stool 的源码树可以放在任意目录：`integration/runtime_env.sh` 按当前目录（`$PWD`）设置路径，运行时也没有源码 SHA 断言。下面用 `S` 表示源码树根。

1. **环境**：按 [`artifacts/physical_environment/`](../../artifacts/physical_environment/README.md) 建 conda 环境 `internutopia311`、装 Isaac Sim 5.1.0、准备 ffmpeg 9.0.2（必须带 libx264：worker 启动时会用 `-c:v libx264` 试编码一帧）。

2. **重建源码树并核 SHA**（在仓库根目录执行）：

   ```bash
   S=/path/to/stool_physical
   python3 roboassemblybench/scripts/prepare_physical_reference.py stool --dest "$S"
   ```

   必须输出 `problems: 0`。请在 Linux 上重建：overlay 中有 30 个 Git 跟踪的 `._*` 文件，macOS 自带的 tar 会把它们当作扩展属性吞掉（脚本用 Python `tarfile`，不受影响）。`compat/` 是其他路线的冻结副本，部分符号链接指向不存在的绝对路径；验证器、worker 和任务代码都不引用它。

3. **源码树内的三个运行时目录**（`runtime_env.sh` 和 `integration/plumbers_validate.py` 从这里加载；版本与 wheel SHA-256 见 [`stool_private_runtime.json`](../../artifacts/physical_environment/stool_private_runtime.json)）：

   ```bash
   ENV=artifacts/physical_environment
   PY="$(conda run -n internutopia311 which python)"
   # 锁定的数值依赖：numpy 1.26.4、scipy 1.17.1、trimesh 4.12.2、python-fcl 0.7.0.6 等 11 个
   "$PY" -m pip install --no-deps --require-hashes --only-binary=:all: \
       --target "$S/.deploy_parts/locked_runtime" -r $ENV/stool_locked_runtime.txt
   # pytest 8.3.5、pluggy 1.5.0、iniconfig 2.0.0
   "$PY" -m pip install --no-deps --require-hashes --only-binary=:all: \
       --target "$S/.runtime_python" -r $ENV/stool_runtime_python.txt
   # ffmpeg / ffprobe
   mkdir -p "$S/.deploy_parts/runtime_bin"
   for x in ffmpeg ffprobe; do ln -s "$(conda run -n physical_ffmpeg which $x)" "$S/.deploy_parts/runtime_bin/$x"; done
   ```

   `--require-hashes` 保证装上的是验证时的同一批 wheel。worker 在锁定模式下还会断言 numpy、scipy、trimesh、fcl 的版本。不要用 `integration/install_private_runtime.py`：它装的是同一批 wheel，但会把带本机路径的清单写回受跟踪的 `integration/private_runtime_manifest.json`。

4. **改两个脚本里的环境值**：

   | 文件 | 原值 | 改成 |
   |---|---|---|
   | `$S/integration/runtime_env.sh` | `ISAAC_SIM_ROOT=/home/user/isaacsim-5.1.0` | 本机 Isaac Sim 目录 |
   | 同上 | `CONDA_EXE=/Embodied_AI/miniconda3/bin/conda` | 本机 conda |
   | 同上 | `PYTHONPATH` 末尾的 `/home/user/beam_integration_20261010_upstream6f57dae/runtime_tools/geometry_deps` | 可删去（锁定目录里已有 python-fcl，且排在前面） |
   | `$S/outputs/stool_physical/run_smoke.sh`（从 `artifacts/stool_physical/run_tools/` 复制） | `cd /home/user/work/zhaoheng/stool_physical_20261120` | `cd "$S"` |
   | 同上 | `/Embodied_AI/miniconda3/envs/internutopia311/bin/python` | 本机 `internutopia311` 的 python |

   `integration/plumbers_validate.py` 也会往 `PYTHONPATH` 追加同一个 Beam `geometry_deps` 路径。路径不存在时 Python 会忽略它，不用改。改过 `runtime_env.sh` 后，`prepare_physical_reference.py stool --verify-only` 会报这一个文件不同，属预期。

   ```bash
   mkdir -p "$S/outputs/stool_physical"
   cp artifacts/stool_physical/run_tools/run_smoke.sh artifacts/stool_physical/run_tools/watch.sh "$S/outputs/stool_physical/"
   ```

5. **全程运行**（约 1.5 小时，需空闲内存约 16 GiB 以上）：

   ```bash
   cd "$S"
   N=smoke_repro; mkdir -p outputs/stool_physical/$N
   CUDA_VISIBLE_DEVICES=0 nohup bash outputs/stool_physical/run_smoke.sh $N 9000 > outputs/stool_physical/$N/launcher.log 2>&1 &
   bash outputs/stool_physical/watch.sh outputs/stool_physical/$N/native/validation.log   # 按阶段跟进度
   ```

   `run_smoke.sh NAME 超时秒数` 会先 `source integration/runtime_env.sh`，再以 `--seed 4906 --layout-seed 4906`、场景 `taoyuan_grscenes_tabletop` 运行 `integration/plumbers_validate.py`，输出写到 `outputs/stool_physical/NAME/native/`。默认只开闭爪两份 trace，可加环境变量 `MOTION=1`、`HOLD=1`、`HOLDFAIL=1`。

6. **核对**：`native/validation_summary.json` 应为 `num_successful: 1`，`worker_exit_code: 0`。把它与 [`runs/smoke38/native/validation_summary.json`](../../artifacts/stool_physical/runs/smoke38/native/validation_summary.json) 对照：53,191 步、`success-criteria-met`、九件误差。仿真是确定性的，逐步相同只在原 GPU（NVIDIA L20，驱动 580.178.04）上验证过；换硬件后若有差异，以验收指标为准。

`run_tools/launch_round.py` / `start_round.sh` 是原批次带额度的正式启动器，会读写运行记录，复现时不要用。

## 5. 验收口径

- 九件到原运行时目标，各件用原位置容差和 0.15 rad。
- 纯物理：无固定抓取/装配连接，无运行时位姿或速度写入，不关碰撞或重力，无运动学搬运。
- 全部真实释放：两爪全开，无接触附着。
- 双臂回 home 后，全件无载静置窗通过。
