# Stool 修复源码与版本入口

本仓库以独立运行树保留成功版本，避免直接替换已有 Beam/批量采集共用控制文件。所有历史路径只用于追溯；新运行由 [专用启动器](../../scripts/stool_assisted/run.py)定位当前 checkout。

## 查看原始源码

```bash
python scripts/stool_assisted/run.py --prepare
```

展开后从以下位置进入：

- **任务 recipe：** `outputs/stool_assisted_reference/stool_repository/roboassemblybench/tasks/fabrica_stool_circular_ur5e_staged/recipe.yaml`。
- **Stool 辅助编译器：** `.../stool_repository/roboassemblybench/core/fabrica_assisted_compiler.py`，负责专属装配合同、右臂扶持时机/姿态/协作路径及任务阶段适配。
- **实际运动技能：** `.../stool_repository/toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py`，文件名保留旧兼容名称；包含 Stool 的抓取、搬运、安装与退避控制。
- **抓取及辅助状态：** 同目录的 `beam_assisted_visual_grasp.py`、`beam_assisted_state.py`，包含 Stool 专属分支；不能据文件名认定运行的是 Beam。
- **真实 worker：** `outputs/stool_assisted_reference/tools/stool_worker.py`。
- **双臂/夹爪/持件几何：** 相邻 `stool_geometry_guard.py`、`stool_export_geometry.py`、`stool_sweep_model.py`。
- **原生状态及启动图：** 相邻 `stool_raw_evidence.py`、`stool_startup_evidence.py`。
- **原历史正式启动与收尾工具：** 相邻 `launch_stool.py`、`close_stool_run.py`；这些保留旧服务器账本语义，供追溯使用，**新机器不要直接调用它们**。新执行入口见 [RUN.md](RUN.md)。

无需展开也可阅读[相对本次主线的源码差异](../../artifacts/stool_assisted/changes_from_main.patch)。差异文件用于阅读，不用于覆盖当前主线；其中也保留旧测试和历史文档的引用。

## 精确原件

- [完整源码冻结包](../../artifacts/stool_assisted/S24_source_snapshot.tar.gz)：914 个原始文件，SHA256 `d4f7d123d3bfb2db9e6b86db6460c02d6b292bc0d065ba1d0791b6f937dae6e3`。
- [完整工具冻结包](../../artifacts/stool_assisted/S24_tools_snapshot.tar.gz)：41 个工具/fixture 文件，SHA256 `0b3fd74b697cfde54d7b3cb253ef506edd09f6cc99e516c7edc8c2e64ebf7148`。
- [逐文件原始绑定](../../artifacts/stool_assisted/source_binding.json)、[原实际运行绑定](../../artifacts/stool_assisted/runtime_source_binding.json)、[当前发布清单](../../artifacts/stool_assisted/release.json)。

控制源码签名为 `409ed6b38c9839978763752e38518064fdb957ab3e628d807b1a87baa1ce1fc8`，控制工具签名为 `41c229803688c7a2bcd8540de8b95a5925b9a1460b68d3ceb835809e8d75bcd8`。本次整理没有修改这些原件，也没有把未完成的性能候选设为默认。

## 资产与环境边界

Stool 资产新增于 `roboassemblybench/assets/Fabrica/canonical_7_bundles/task_bundles/fabrica_official_stool_circular_fixture_pickup_sdf512_v1/`。UR5e、Robotiq、桌面等复用仓库既有资产，所有发布依赖均由 `release.json` 校验。原运行 34 个 composed USD layer 的内容与仓库文件逐项核对。

冻结源码不包含 Isaac Sim 本体、Conda 环境或旧服务器全部大日志。原始历史完整状态证据仍在原服务器及另行保存的备份中；仓库的原结果摘要、身份、录像与完成图用于查看本次修复成果，不冒充那几 GB 原始 trace 的完整替代品。
