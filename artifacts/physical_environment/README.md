# 运行环境锁定（Beam v239 / Stool P09 共用）

两条路线验证通过时的运行环境。为了尽量得到相同结果，请按下面的版本搭建。

## 机器与系统

| 项 | 验证时的值 |
|---|---|
| GPU | NVIDIA L20（46 GB），驱动 580.178.04，CUDA 13.0 |
| CPU | AMD EPYC 9554 |
| 系统 | Ubuntu 22.04.5 LTS，内核 6.8.0-138-generic，glibc 2.35 |
| Isaac Sim | 5.1.0（build `5.1.0-rc.19+release.26219.9c81211b.gl`） |
| Python | 3.11.13（conda 环境 `internutopia311`） |
| ffmpeg | 9.0.2（conda-forge，带 libx264 / openh264） |

仿真结果逐步一致只在上述配置上验证过。换 GPU 或驱动大概率仍能完成装配，但数值可能不同，不能保证与仓库记录逐行一致。

## conda 环境 `internutopia311`

| 文件 | 用途 |
|---|---|
| `internutopia311_conda_explicit.txt` | `conda list --explicit` 导出的 conda 包（含 Python 3.11.13），按 URL 精确还原 |
| `internutopia311_pip_requirements.txt` | 可直接安装的 pip 包清单（104 个，例如 numpy 2.4.6、scipy 1.17.1、torch 2.14.0） |
| `internutopia311_pip_freeze.txt` | `pip freeze --all` 原样输出，仅供对照 |
| `internutopia311_environment.yml` | `conda env export --no-builds`，仅供对照 |

```bash
conda create -n internutopia311 --file artifacts/physical_environment/internutopia311_conda_explicit.txt
conda activate internutopia311
pip install -r artifacts/physical_environment/internutopia311_pip_requirements.txt
```

原环境里还有一个可编辑安装 `-e /home/user/RoboAssemblyBench`，以及 `packaging`、`pip` 两条指向 conda 本地构建目录的条目（原样保留在 `pip_freeze.txt` 中）。requirements 文件去掉了这三行：后两个包由 conda 清单提供。运行时两条路线都把自己的源码树放在 `PYTHONPATH` 最前面，Beam 的 `checked_worker.py` 还会断言关键模块从源码树加载，所以不需要装这个可编辑包。

运行前加载 Isaac Sim：`source $ISAAC_SIM_ROOT/setup_conda_env.sh`。

## 两条路线各自的附加依赖

- **Beam**：`python-fcl==0.7.0.8`，单独装到一个目录，并加入 `PYTHONPATH`（启动脚本里是 `$R/runtime_tools/geometry_deps`）：

  ```bash
  pip install --no-deps --target "$R/runtime_tools/geometry_deps" python-fcl==0.7.0.8
  ```

  另需 ffmpeg 放在 `$R/runtime_tools/bin/ffmpeg`。
- **Stool**：锁定的数值依赖、pytest 与 ffmpeg 都放在源码树内，步骤见 [Stool 复现说明](../../docs/stool_physical/HANDOFF.md) 第 4 节。锁定版本与 wheel SHA-256 记在 `stool_private_runtime.json`。

ffmpeg（两条路线相同版本）：

```bash
conda create -n physical_ffmpeg -c conda-forge ffmpeg=9.0.2 x264 openh264
```

## Isaac 官方资产

Beam 的 worker 配置引用 `${ISAAC_ASSETS_ROOT}/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd`，请把 `ISAAC_ASSETS_ROOT` 设为 Isaac Sim 5.1 官方资产根目录。
