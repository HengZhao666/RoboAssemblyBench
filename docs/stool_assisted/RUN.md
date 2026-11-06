# 执行修复后的 Stool 辅助装配

总入口：[STOOL_REPAIR_README.md](../../STOOL_REPAIR_README.md)。先看已有录像可直接进入 [RESULTS.md](RESULTS.md)，不必重跑。

## 准备环境

正式仿真需要 Linux、NVIDIA GPU、Isaac Sim standalone **5.1.0**、Python **3.11** 项目环境，以及 PATH 中的 `ffmpeg`、`ffprobe`、`nvidia-smi`。需要根 README 中的基础环境依赖；已有 Beam 环境可以复用。建议至少 32 GB 内存，并为每次完整运行预留 **30 GB 可用输出空间**。历史完整运行约 3 小时 25 分钟，视频时长并不等于实际运行耗时。

```bash
git clone --branch main https://github.com/HengZhao666/RoboAssemblyBench.git
cd RoboAssemblyBench
```

如未准备项目环境，先按[根 README 环境安装部分](../../README.md#41-isaac-sim--数据生成环境)安装 Isaac Sim 并运行 `bash setup_conda.sh`，环境名使用 `internutopia311`；随后更新 `environment.yml`。在已经正常工作的环境中执行：

```bash
conda activate internutopia311
export ISAAC_SIM_ROOT=/你的安装路径/isaac-sim-5.1.0
python -m pip install -r requirements/stool_assisted.txt
```

仓库已包含此版本需要的 stool bundle 和复用的 UR5e/夹爪资产。首次加载官方环境可能需要网络。Isaac Sim 本体、驱动和 Python 环境仍需自行安装。

## 展开和检查入口

```bash
# 只校验并展开源码/工具，macOS 也可执行；不启动仿真。
python scripts/stool_assisted/run.py --prepare

# 在 Linux / Isaac Python 环境执行真实构建检查；不推进物理。
bash roboassemblybench/scripts/run_stool_assisted.sh --check
```

`--prepare` 展开原始 914 个源码文件及 41 个工具/fixture 文件，并核对逐文件 SHA。源码位于 `outputs/stool_assisted_reference/stool_repository/`，工具位于相邻 `tools/`。资产连接只指向当前 checkout，不要求旧服务器的绝对目录或旧账本。

`--check` 执行实际采集 worker 的配置构建、env0/env1 名称映射及生产预检，并把完整物理配置与成功运行的配置比较。只允许路径迁移和顶层 recipe 指纹元数据变化；抓取、目标、PD 参数、阶段、双臂/持件检查和采样窗口均保持原值。成功返回 `"passed": true`。

如需选择解释器，可设置 `STOOL_PYTHON=/绝对路径/python`；额外的已安装依赖可通过 `STOOL_EXTRA_PYTHONPATH` 指定。不要向该变量加入其他项目源码，以免混用控制模块。

## 正式启动

```bash
bash roboassemblybench/scripts/run_stool_assisted.sh \
  --output-dir outputs/stool_assisted_demo
```

每次使用新的输出目录，已有目录会拒绝覆盖。该命令使用固定 seed `0`、layout seed `4906`、原场景和三路相机，不读取或修改旧服务器的修复次数、暂停账本或历史启动许可。

启动器打印本轮 `startup_review.html` 路径后，打开它查看正面、左腕和右腕三张启动图。确认机械臂、零件、相机正常，在原终端输入 **`yes`** 才开始装配。无交互终端使用：

```bash
bash roboassemblybench/scripts/run_stool_assisted.sh \
  --output-dir outputs/stool_assisted_demo --no-prompt
```

查看本轮图像后，在第二个终端确认同一个输出目录：

```bash
bash roboassemblybench/scripts/run_stool_assisted.sh \
  --approve-startup outputs/stool_assisted_demo
```

图像确认保持原 worker 的 10 分钟期限，绑定到本轮原生物理时刻和真实首条控制指令，旧图像或旧确认文件不能复用。

## 中止与结果

```bash
bash roboassemblybench/scripts/run_stool_assisted.sh \
  --stop outputs/stool_assisted_demo
```

也可在运行终端按 Ctrl+C 请求收尾。等待 worker 编码视频并退出；不要重复启动同一个输出目录。

完成后查看：

- `result_summary.json`：本次新运行的功能结果、原任务结果、终止原因和源码/资产是否保持不变。`functional_success` 不是历史成功结果的复制。
- `episode_0000_live_videos/observation_images_front.mp4`：正面完整录像；同目录有左右腕完整录像。
- `worker.log`、`collect_results.json`：失败阶段和原始任务结果。
- `preflight.json`、`source_binding.json`、`runtime_source_binding.json`：本次构建和实际运行版本。
- `scene_geometry.json`、原生状态及各类 trace：定位问题所需的原始证据。

`strict_independent_terminal_verified` 在新入口的汇总中保持 false：历史成功版本的完整后验结果存放在交付原件中，新运行并未自动执行那套完整独立复核。不要把新入口的静态构建通过或进程退出当作新一次完整装配成功。

为保留原成功流程，本入口仍保留原三路图像帧和详细日志，尚未采用 S25/S26 的提速候选。清理时保护活跃运行；仅删除已有完整录像可重建且无引用的缓存，大原始日志先无损归档并核验后再删明文。
