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

验证时的完整环境已锁定在 [artifacts/physical_environment/](artifacts/physical_environment/README.md)：

- 机器与版本：NVIDIA L20（驱动 580.178.04）、Ubuntu 22.04、Isaac Sim 5.1.0（build `5.1.0-rc.19`）、Python 3.11.13
- conda 环境 `internutopia311`：conda 精确清单与 104 个 pip 包
- ffmpeg 9.0.2（带 libx264）
- Beam 用的 python-fcl 0.7.0.8；Stool 源码树内的锁定依赖（按 wheel SHA-256 安装）

两条路线的放置方式不同：

- **Beam**：启动链对 worker 配置、资产清单和关键模块做了 SHA 断言，这些文件里写的是绝对路径，所以必须按原路径 `/home/user/beam_integration_20261010_upstream6f57dae/` 摆放，只改两个 shell 脚本里的环境值。
- **Stool**：源码树可放在任意目录，只改两个脚本里的环境值，并在树内建好三个运行时目录。

## 第三步：运行并核对

先跑短冒烟，与仓库中保存的结果逐行比对（Beam 用 `artifacts/beam_physical/compare_progress.py`），再跑全程。仿真是确定性的：源码相同、配置相同时，结果应与仓库记录逐步一致。不同 GPU 或驱动可能带来数值差异。
