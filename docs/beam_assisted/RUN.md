# 运行修复后的 Beam 辅助版

此入口运行 **`fabrica_beam_ur5e_assisted`**，使用与 A44 相同的物理、控制、seed 0、layout seed 505 和三路相机配置。普通 `generate_fabrica_canonical_ur5e_demo.sh beam` 选择的是另一套 staged 流程，不能替代本入口。

## 1. 准备环境（每台机器一次）

需要 Linux、可运行 Isaac Sim 的 NVIDIA GPU、**Isaac Sim standalone 5.1.0**、Python 3.11 项目环境，以及 PATH 中的 `ffmpeg`、`ffprobe`、`nvidia-smi`。首次加载官方场景需要联网。建议预留至少 32 GB 内存和 15 GB 输出空间。macOS、其他 Isaac 版本和随机布局不在本入口的验证范围内。

```bash
git clone --branch main https://github.com/HengZhao666/RoboAssemblyBench.git
cd RoboAssemblyBench
export ISAAC_SIM_ROOT=/你安装的路径/isaac-sim-5.1.0
```

如果还没有本项目的 Python 环境，先按根 [README 的环境安装部分](../../README.md#41-isaac-sim--数据生成环境)执行 `bash setup_conda.sh`，输入环境名 `internutopia311`，再执行 `conda env update -n internutopia311 -f environment.yml`。已有正常项目环境则直接激活。最后安装辅助版依赖覆盖：

```bash
conda activate internutopia311
python -m pip install -r requirements/beam_assisted.txt
bash roboassemblybench/scripts/run_beam_assisted.sh --check
```

`--check` 不推进仿真：核查源码、已随仓库提供的 Beam/UR5e 资产、依赖、GPU、Isaac 文件，以及真实任务构建→框架名称映射→生产预检。成功输出 `"passed": true`。不需要下载旧服务器目录、旧授权账本或复制旧 `launch_gate.json`。依赖版本和实际导入位置会写入预检结果。

## 2. 正式运行

在已激活的环境中，从仓库根目录执行：

```bash
bash roboassemblybench/scripts/run_beam_assisted.sh --output-dir outputs/beam_assisted_demo
```

每次使用新的输出目录；已有目录会拒绝覆盖。启动器自动找到当前源码和资产，按当前路径重新生成配置绑定，启动原辅助 worker 和独立几何检查。它不会读取原服务器的实验次数或文档同步收据。

启动后会打印 `startup_review.html` 的位置。打开其中的本轮正面、左腕、右腕三张图，确认机械臂、零件和相机正常，然后在运行终端输入 **`yes`**。无交互终端可加 `--no-prompt`，查看图后在另一终端明确确认：

```bash
bash roboassemblybench/scripts/run_beam_assisted.sh --approve-startup outputs/beam_assisted_demo
```

确认前机械臂不会开始装配。远程服务器可把该目录中的 HTML 和三张 `startup_*.png` 下载到本机一起查看。任务包含完整碰撞、持件和终态检查，运行时间远长于视频时长；不要因暂时没有终端输出就重复启动。

需要中止时，在运行终端按 Ctrl+C，或执行：

```bash
bash roboassemblybench/scripts/run_beam_assisted.sh --stop outputs/beam_assisted_demo
```

## 3. 看结果和视频

所有本轮文件都在指定输出目录：

- `result_summary.json`：`functional_success: true` 表示五件辅助装配、双臂回位、原任务稳定窗口及独立运行几何检查通过，且运行源码和资产未变。
- `episode_0000_live_videos/`：正面、左腕、右腕三份完整 MP4。
- `worker.log`、`geometry_monitor.log`、`collect_results.json`：具体失败位置及原任务结论。
- `preflight.json`、`runtime_import_binding.json`：实际路径、依赖与运行身份；向他人排查时一起提供。

默认编码后删除可再生成的 PNG 帧缓存，保留三份完整视频和原始状态记录；确需缓存可加 `--keep-video-frames`。

这是保留固定连接与安装支撑的**辅助装配**。`strict_independent_terminal_verified` 保持 false：本次没有改写原来的末尾独立 raw 采样缺口，也不把功能完成称作撤除支撑后的自然稳定。

## 4. 部署改了什么

新增公开入口 [run_beam_assisted.sh](../../roboassemblybench/scripts/run_beam_assisted.sh) 和 [run.py](../../scripts/beam_assisted/run.py)；原控制、抓取、安装、相机和成功门保持 v35。小型 [runtime 适配层](../../scripts/beam_assisted/runtime/) 只替换旧工具中固定的源码/输出路径和导入定位。依赖覆盖固定 `python-fcl==0.7.0.8`。源码、资产及参考物理配置由 [portable_release.json](../../artifacts/beam_assisted/portable_release.json) 绑定。

原服务器工具、v35 封存包和 A44 原始结果仍完整保留，作为历史证据。部署说明的修改不影响运行源码身份；实际物理配置变化仍会使预检失败。

## 验证记录

2026-10-26：12 项部署测试通过。独立 GitHub checkout 的完整运行验证进行中；最终结果将在完成后补到本节。已有服务器 A45 使用 v35 完成五件辅助装配与双臂回位；它不替代新入口的独立目录验证。
