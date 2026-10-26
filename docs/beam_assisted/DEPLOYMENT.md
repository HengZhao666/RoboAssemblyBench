# Beam assisted 额外运行依赖与资产核查

> 本页保留 2026-10-21 的原部署审计。新可移植入口及当前验证范围见 [RUN.md](RUN.md)；下文的服务器绝对路径属于历史记录，不是新入口的运行前提。

2026-10-21。来源：`pcl-server` 只读文件盘点、SHA256、静态 Python METADATA / Conda 包元数据；未运行测试、pip、Isaac 或仿真。原仓库对比基线为 `2739e27e66418048d79097376648af1b389a2b68`。完整机器清单见 [runtime_dependencies.json](../../artifacts/beam_assisted/runtime_dependencies.json)，新增资产定位见[复制清单](../../artifacts/beam_assisted/additional_asset_copy_manifest.json)。

## 项目代码之外的运行依赖

- `R/runtime_tools/bin` 只有 `ffmpeg` symlink，指向 `/Embodied_AI/miniconda3/envs/fabrica/bin/ffmpeg`。这是第三方 ELF，不存在额外的自有 wrapper 漏传。实际二进制 SHA256 为 `cb98c82108d5caab751f370441445f778038dcab1507c2d34ab842fcdf838328`；Conda 元数据为 `ffmpeg 9.0.2 / gpl_h9b364d9_900`，conda-forge 包来源和包 SHA 已记录。新环境应安装受支持的 FFmpeg 并映射 PATH，不提交服务器绝对 symlink 或完整 Conda 环境。
- `R/runtime_tools/geometry_deps` 共 23 文件，仅包含 `python-fcl 0.7.0.8`、`fcl` 扩展及附带动态库。这是安装出的第三方包，无第一方脚本。完整 CAD 核查的 `EntityGeometryChecker` 动态使用 `fcl` 与 `scipy.spatial.ConvexHull`，不能因目录不进 PR 就省略这些依赖。
- `R/test_dependencies` 共 252 文件，是 `pytest 7.4.4`、`pluggy 1.6.0`、`packaging 26.3`、`iniconfig 2.3.1` 及字节缓存。常规 `requirements/test.txt` 声明 pytest7.3.1，与当前额外 PYTHONPATH 上的版本不同；应按此记录说明实际运行环境，不将整个第三方安装目录提交为项目代码。
- `internutopia311` 环境静态元数据记录 NumPy2.4.6、SciPy1.17.1、Pydantic2.13.5、Pillow12.3.0。这里只证实当前磁盘候选包版本，没有导入这些包，也未证明 Isaac 启动后同名模块的优先顺序；不能把本清单冒充 A44 当时完整锁定环境或已验证的全新安装组合。
- 原生产入口仍需要 Isaac Sim5.1.0、Conda环境与 R/P/A 布局。A44实际配置使用 Isaac 安装内 UR5e URDF，路径与 SHA 已记录；Isaac安装属于外部运行时。

## 实际 Beam 资产与原仓库关系

- Beam canonical bundle 共 21 文件、7,983,964 bytes，包括五个零件、实际夹具、光学板、场景、抓取/计划元数据与原资产验证记录。21 文件均不在原基线 Git 中，也不在889源封存中。
- UR5e/Robotiq wrist-mount 入口来自 `fabrica_ur5e_cooling_optical_board_black_fullbundle_sdf001`，包共470文件、221,290,703 bytes。全部470项均与原基线对应 Git blob 相同，包括包中的3个Python脚本。保留原仓库内容即可，不能当新增 Beam 改动描述。
- `open_in_isaacsim.py` / `.sh` 两项第一方外部入口也与原基线完全相同。额外 symlink 文件 `Fabrica/optical_board.usda` 与 `Fabrica/README.md` 同样与原基线相同。
- canonical `SHA256SUMS`、`HANDOFF_README.md` 和共享 Franka Lula URDF 共3项原基线未跟踪。连同 Beam21，24项待复制文件及真实服务器源路径、仓库目标路径、SHA见 `additional_asset_copy_manifest.json`，合计8,028,220 bytes。服务器还有1字节 `canonical_7_bundles/.gitkeep` 未跟踪；加入已有实文件的目录不是运行必需，可明确列为目录标记或一并保留。
- 服务器 A 共22个 symlink，文件级比较记录在 `server_asset_symlinks.json` / 完整JSON中。只读取共享资产内容和元数据，没有加载另一条路线的物理机制或历史。
- 所读 USDA 的字面相对引用均可在服务器解析，入口机器人引用 UR5e与Robotiq配置也可解析；本轮没有使用 pxr 解码二进制 USD 的全依赖图。完整 UR5e 包字节与基线相同提供保留依据，不能将此静态检查称为干净环境启动成功。

## 获取合同与边界

根 `README.md` 已给出下载器 `roboassemblybench/scripts/download_repro_assets_from_hf.py` 的显式命令：HF dataset `baiyu858/InternUtopia-repro-assets`，include canonical7 bundles、UR5e cooling bundle、Franka bundle和场景资源。下载器支持 `--revision` 和 `--include`；其默认 include 列表不含 UR5e cooling bundle，复现应使用 README 的显式 include。此轮不联网下载 HF，不声称其当前分支头与服务器字节一致；用记录的 SHA逐文件核验，固定确认过的revision后才能称资产版本锁定。

若本PR只直接加入Beam21资产，canonical `SHA256SUMS` / `HANDOFF_README` 仍描述全部7任务：其他6任务的 bundle 需下载后才能对整份 SHA256SUMS 执行完整校验。不能以本PR Beam资产齐全声称7任务全部资产已经纳入。服务器原始验证文件仅保留其既有结论，不形成新仿真证据。

服务器完整工具在 `outputs/beam_assisted_20261015/tools/`。这些脚本保留原R/P/A部署路径及原账本合同；本次没有改成通用启动器，也不自动修改现有服务器环境。先按根 `BEAM_ASSISTED_PR_HANDOFF.md` 核文件与范围，再在明确指定的任务/运行授权内操作。
