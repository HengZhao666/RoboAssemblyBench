# 纯物理装配复现：Beam 与 Stool circular（双 UR5e）

两条**纯物理**装配路线都已完成整件装配：摩擦抓持、真实释放，不用固定连接、位姿写入、速度清零，也不关闭碰撞或重力。复现所需的全部代码都在本仓库里，每条路线一份复现说明。

| 路线 | 已验证结果 | 复现说明 | 代码与证据 |
|---|---|---|---|
| **Beam**（5 件） | v239：五件装入，双臂回位，最终无载验证第 240 步通过，共 60,524 步 | [docs/beam_physical/HANDOFF.md](docs/beam_physical/HANDOFF.md) | [artifacts/beam_physical/](artifacts/beam_physical/) |
| **Stool circular**（9 件） | P09：九件装入，双臂回位，无载静置通过，共 53,191 步 | [docs/stool_physical/HANDOFF.md](docs/stool_physical/HANDOFF.md) | [artifacts/stool_physical/](artifacts/stool_physical/) |

仓库里已有的 [Beam 辅助版](BEAM_REPAIR_README.md) 和 [Stool 辅助版](STOOL_REPAIR_README.md) 使用虚拟工装，是另一条路线，与本页无关。

## 第一步：重建源码树

两条路线运行时的源码树与 `main` 大部分相同。仓库只存放不同的文件（overlay）和全部文件的 SHA-256 清单，用下面的脚本重建：

```bash
git clone https://github.com/HengZhao666/RoboAssemblyBench.git && cd RoboAssemblyBench
python3 roboassemblybench/scripts/prepare_physical_reference.py beam    # -> outputs/beam_physical_reference/
python3 roboassemblybench/scripts/prepare_physical_reference.py stool   # -> outputs/stool_physical_reference/
```

脚本只用 Python 标准库（≥3.8），可用 `--dest` 指定目录、`--verify-only` 只做核对。两条命令都输出 `problems: 0`，才说明重建的树与验证通过时运行的源码逐字节相同（Beam 904 个文件；Stool 6,475 个文件加 78 个符号链接）。请在 Linux 上重建和运行。

## 第二步：准备运行环境

两条路线共用：

- Isaac Sim 5.1.0（设 `ISAAC_SIM_ROOT`，运行前 `source $ISAAC_SIM_ROOT/setup_conda_env.sh`）
- conda 环境 `internutopia311`（Python 3.11），按仓库 `environment.yml` 和 `requirements/` 安装
- python-fcl、ffmpeg、NVIDIA GPU 与 Vulkan ICD

各路线还需要的依赖、启动脚本里要替换的路径，以及运行和核对结果的方法，见各自的复现说明。

## 第三步：运行并核对

先跑短冒烟，与仓库中保存的结果逐行比对，再跑全程。仿真是确定性的：源码相同、配置相同时，结果应与仓库记录逐步一致。不同 GPU 或驱动可能带来数值差异。
