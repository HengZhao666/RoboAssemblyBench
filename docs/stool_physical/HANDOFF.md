# Stool circular 纯物理装配 · 复现交接（P09 / R05）

整理日期 2026-11-26。本页面向接手同事：说明最新结果、复现所需的全部代码在仓库中的位置，以及复现步骤。总入口见 [PHYSICAL_ASSEMBLY_HANDOFF.md](../../PHYSICAL_ASSEMBLY_HANDOFF.md)。本路线与仓库里已有的 [Stool 辅助装配版](../../STOOL_REPAIR_README.md)（S24，虚拟工装）是两条独立路线，代码、配方和验收都不同。

## 1. 当前结果

- **正式记录 R05（第5/5轮，包 P08）核验成功**，2026-11-24 10:41 CST 启动：九件全部纯物理装入，`success-criteria-met`，53,580 步（约 223 s 仿真时间），墙钟 5,423 s，零超时、零恢复、运行时约束违规 0；双臂空手回位，最终无载静置 30 步通过。13 次抓取全部为 `pure_physical_grasp`（无固定关节、位姿写入或锁定，抓取时保持碰撞）。与同源码 smoke35 逐步相同（确定性复现）。
- 逐件误差 / 容差：帽 0 0.35/18 mm、腿 1 0.99/50.5、底板 0.16/19.2、腿 3 0.25/50.5、帽 4 0.68/18、腿 5 0.81/50.5、帽 6 1.80/18、腿 7 0.19/50.5、帽 8 0.50/18 mm，姿态均 ≤0.041/0.15 rad；相对底板原名义装配位独立复核九件均 ≤1.54 mm、≤0.031 rad。
- **R05 之后的根治版 P09（推荐复现这一版）**：按用户"根治"指令去掉 R05 用到的灰色暂放台，改为光学板碰撞用同顶面长方体、板摩擦 0.8/0.6。不计轮次的全程验证 **smoke38 通过完整合同**：九件 `success-criteria-met`，53,191 步，零超时/恢复，双臂回位，无载静置 30/30，验收误差最大 1.48 mm / 0.039 rad，相对底板名义位 ≤1.59 mm / 0.043 rad，墙钟 5,347 s。正式记录仍为 R05（P08）；P09 计入正式轮次需新的人类指令。
- 账本状态 `R05_verified_success_stopped__root_fix_validated_smoke38`，5/5 轮已用、成功即停。**继续仿真需新的人类指令。**

交付时需说明的改动（相对原 Stool 合同）：帽件成功目标在取件前按所插腿的实际位姿重定位（容差不变）；帽件两指严格接触以 PhysX 实测接触点配对并计入 1 mm SDF 余量；帽件"边夹—放直—深 10 mm 重抓"换抓流程；腿 1 / 帽 0 / 帽 8 限开度松爪；P09 中光学板碰撞改为长方体（顶面不变，孔被填平，Stool 不用孔）、摩擦 0.8/0.6（P08/R05 则是新增 108×108×3 mm 被动静态暂放台）。

R05 三路视频的 SHA 记在 [`runs/R05_20261124_104131_P08/review.json`](../../artifacts/stool_physical/runs/R05_20261124_104131_P08/review.json)；视频本身在服务器 `$SP/rounds/R05_20261124_104131_P08/native/episode_0000_live_videos/`，Mac 工作区 `outputs/stool_physical_R05_20261124/` 与 `outputs/stool_physical_smoke38_20261124_root_fix/`，未放入仓库。

## 2. 仓库里有什么

所有文件在 [`artifacts/stool_physical/`](../../artifacts/stool_physical/)：

| 路径 | 内容 |
|---|---|
| `source_overlay_P09.tar.gz` | 服务器工作树（= Plumbers 集成提交 `6f1de1d` + P09 改动）中与本仓库 `main` 不同的 4,008 个文件和 78 个符号链接，主要是集成框架 `integration/`、冻结兼容树 `compat/`、Plumbers 物理与 Stool 配方。其余 2,467 个文件（含全部用到的资产）与 `main` 逐字节相同，不重复存放 |
| `source_manifest.json` | 服务器工作树全部 6,475 个文件的 SHA-256 及来源（`main` / `overlay`），外加 78 个符号链接 |
| `changes_P09_vs_6f1de1d.patch` | Stool 纯物理相对 Plumbers C01 框架（`6f1de1d`）的全部改动，9 个文件，可读 diff（SHA-256 `25e2c78c6d0ee736…`） |
| `packages/P01…P09` | 各候选包的 `delta_from_6f1de1d.patch` 与源文件 SHA；R01–R05 分别用 P03、P04、P05、P06、P08，P07 为 R05 前最佳已知，P09 为根治版 |
| `run_tools/` | `run_smoke.sh`（不计轮次的全程/调试运行）、`start_round.sh` + `launch_round.py`（正式轮，核账本额度与成功即停）、`watch.sh`（按阶段跟日志）、`py.sh`（带运行环境的离线 Python）、各 `*_probe.py`/`*chk.py` 离线检查与 `patch01…13.py` 补丁脚本、`start_R0*.log` |
| `ledger/ledger.json` | 唯一账本快照（2026-11-26）：授权、验收定义、各轮、冒烟记录、人类指令历史 |
| `runs/R01…R05/`、`runs/smoke35…38/` | 每轮 `review.json`、`launch.json`、`launcher.log`、`native/validation_summary.json` 等小文件 |
| [`roboassemblybench/scripts/prepare_physical_reference.py`](../../roboassemblybench/scripts/prepare_physical_reference.py) | 用 `main` + overlay 重建完整工作树并逐文件核 SHA |

路线全过程（smoke01–38 的物理原因、每次改动、R01–R05 失败分析）：[服务器 Stool 交接原文（2026-11-24）](reference/STOOL_HANDOFF_server_20261124.md)；任务合同：[CONTRACT.md](reference/CONTRACT.md)。

## 3. 代码入口（P09 改动的 9 个文件）

全部改动对 Plumbers 行为保持不变，Stool 通过配方开关启用：

- `roboassemblybench/tasks/fabrica_stool_circular_ur5e_physical/recipe.yaml`、`annotation.yaml`：配方 `fabrica_stool_circular_ur5e_physical`（继承 `fabrica_stool_circular_ur5e_staged`），含左右手指垫刚度/阻尼、臂 PD（kp 640000 / kd 11313.7）、`physical_seat_before_release`、`physical_rebase_targets_to_socket`、`physical_cap_regrasp`、`physical_servo_reference_point_parts`、`physical_limited_release`、`physical_board_collision: boundingCube`、`physical_board_friction` 等开关。
- `roboassemblybench/core/plumbers_physical.py`：接触盒对齐抓取、座入后再释放、帽件重定位、换抓阶段、限开度松爪、实测接触点严格配对（`strict_force_point_surface_margin` 1 mm）等编译逻辑。
- `roboassemblybench/core/fabrica_canonical.py`：连续物理分支扩到 `stool_circular`（`_GENERIC_PHYSICAL_ASSEMBLIES`），光学板碰撞近似 `physical_board_collision` 与摩擦 `physical_board_friction`；`plumbers_support.py`：`add_terminal_joint_home` 只在存在支撑撤离引用时改写最后撤离。
- `internutopia_extension/tasks/factory_dual_franka_assembly_task.py`：可选零件系 `contact_box_orientation`、`keep_anchor_target`、实测接触点配对的 `strict_force_point_surface_margin`。
- `toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py`：`target_object_reference_point` 伺服/收敛参考点、闭爪 trace 附接触详情；`demo_policy.py`：空闲臂保持允许 Stool；`integration/phase_journal.py`：按实际零件名记录。

## 4. 复现

### A. 在 pcl-server 原地复现

服务器工作树 `S=/home/user/work/zhaoheng/stool_physical_20261120` 当前就是 P09（9 个改动文件的 SHA 已于 2026-11-26 与 `packages/P09_board_root_fix/source_sha256.json` 逐一核对相符）。它是 `6f1de1d` 的部分克隆（`blob:none`），不能 `git commit`，所以每个候选包以补丁 + SHA 清单冻结。

```bash
S=/home/user/work/zhaoheng/stool_physical_20261120
cd $S
# 1) 核对工作树仍是 P09
/usr/bin/python3 - <<'PY'
import hashlib, json
m = json.load(open('outputs/stool_physical/packages/P09_board_root_fix/source_sha256.json'))
print({p: hashlib.sha256(open(p, 'rb').read()).hexdigest() == h for p, h in m.items()})
PY
# 2) 不计轮次的全程运行（约 1.5 小时；NAME 用新的目录名，第二个参数是超时秒数）
N=smoke39_repro; mkdir -p outputs/stool_physical/$N
nohup bash outputs/stool_physical/run_smoke.sh $N 9000 > outputs/stool_physical/$N/launcher.log 2>&1 &
# 按阶段跟进度
bash outputs/stool_physical/watch.sh outputs/stool_physical/$N/native/validation.log
```

- `run_smoke.sh` 先 `source integration/runtime_env.sh`，再以 `--seed 4906 --layout-seed 4906`、场景 `taoyuan_grscenes_tabletop` 运行 `integration/plumbers_validate.py`，输出写到 `outputs/stool_physical/NAME/native/`；默认只开闭爪两份 trace，可加环境变量 `MOTION=1`、`HOLD=1`、`HOLDFAIL=1`。结果对照 `runs/smoke38/native/validation_summary.json`（仿真确定，同源码同配置应逐步相同）。
- 正式轮：`bash outputs/stool_physical/start_round.sh P0x`。`launch_round.py` 会核账本额度（最多 5 轮，已用完）、成功即停、上一轮已复核、磁盘 ≥20 GiB、内存与 GPU，并冻结补丁与 SHA。当前账本会拒绝启动，**需新的人类指令和新的授权记录**。
- 工作树的 `.deploy_parts`、`.runtime_python` 是只读软链，指回 Plumbers 集成树 `/home/user/work/zhaoheng/RoboAssemblyBench-integrated/`（锁定的 Python 依赖 `locked_runtime/`、`runtime_bin/ffmpeg`、`pytest` 等）；`PYTHONPATH` 还用到 Beam 根下的 `runtime_tools/geometry_deps`（python-fcl）。

### B. 在新机器上从本仓库复现（未在其他机器验证）

1. 重建工作树并核 SHA：

   ```bash
   python3 roboassemblybench/scripts/prepare_physical_reference.py stool --dest /path/to/stool_physical_20261120
   ```

   输出 `problems: 0` 才表示 6,475 个文件和 78 个符号链接都与服务器 P09 工作树相同。请在 Linux 上解包和运行：overlay 中有 30 个 Git 跟踪的 `._*` 文件，macOS 自带 tar 会把它们当作扩展属性吞掉（脚本用 Python `tarfile`，不受影响）。`compat/` 下部分符号链接指向服务器绝对路径，只在服务器上可解析；它们是其他路线的冻结副本，不在 Stool 纯物理的运行路径上（未逐一验证）。
2. 运行环境：Isaac Sim 5.1.0（`integration/runtime_env.sh` 里写的是 `/home/user/isaacsim-5.1.0`），conda 环境 `internutopia311`（Python 3.11），锁定依赖：cloudpickle 3.1.2、farama_notifications 0.0.6、gymnasium 1.2.1、ikpy 3.4.2、numpy 1.26.4、packaging 23.2、python_fcl 0.7.0.6、PyYAML 6.0.1、scipy 1.17.1、transforms3d 0.4.2、trimesh 4.12.2（服务器装在 `.deploy_parts/locked_runtime`），外加 pytest 8.3.5 / pluggy 1.5.0 / iniconfig 2.0.0 与 ffmpeg/ffprobe。新机器上需自建 `.deploy_parts/locked_runtime`、`.deploy_parts/runtime_bin`、`.runtime_python` 三个目录，并把 `runtime_env.sh` 中 Beam 根下的 `geometry_deps` 换成本机的 python-fcl 路径。
3. 运行脚本写死了服务器路径，需替换：`/home/user/work/zhaoheng/stool_physical_20261120`（工作树根）、`/Embodied_AI/miniconda3`（conda python 与 git）、`/home/user/isaacsim-5.1.0`、GPU UUID `GPU-e064bda0-b445-2d7f-50d5-c1180deadea2`（`launch_round.py`）。把 `artifacts/stool_physical/run_tools/` 放回工作树的 `outputs/stool_physical/` 后即可用 `run_smoke.sh`。

## 5. 验收口径

原 Stool 合同：九件到原运行时目标、各件原位置容差与 0.15 rad；纯物理（无固定抓取/装配连接、无运行时位姿/速度写入、不关碰撞或重力、无运动学搬运）；全部真实释放（两爪全开、无接触附着）；双臂回 home 后全件无载静置窗通过。离线编译与冒烟不计轮次，正式轮最多 5 次、成功即停（2026-11-20 用户选定）。

## 6. 服务器权威位置

| 内容 | 位置（pcl-server） |
|---|---|
| 工作树 | `/home/user/work/zhaoheng/stool_physical_20261120`（`$S`），运行资料在 `$S/outputs/stool_physical/`（`$SP`） |
| 唯一账本 | `$SP/ledger.json` |
| 候选包 / 正式轮 / 冒烟 | `$SP/packages/`、`$SP/rounds/R0*`、`$SP/smoke*` |
| 框架基线 | Plumbers 集成树 `/home/user/work/zhaoheng/RoboAssemblyBench-integrated`，提交 `6f1de1d98220df888fd69b697432719ce80ded5a`（未推送 GitHub；本仓库以 overlay + SHA 清单保存其完整文件内容） |
| 服务器交接 | `/home/user/beam_integration_20261010_upstream6f57dae/fabrica_tasks/stool_circular_ur5e/HANDOFF.md`（与 Mac 工作区同 SHA） |
