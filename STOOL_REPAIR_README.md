# Stool / 双 UR5e：已修复的辅助装配版本

本入口交付已完成 **9 个零件、183 个阶段**的 circular stool 辅助装配版本：修复夹爪与零件贴合、双臂干涉、安装时的运动跟随和退避路径；右臂在底座放置后进入前视画面左侧的扶持位置，持续覆盖左臂取件、搬运、安装、松爪和退避，最后松开底座。

## 从哪里开始

- **执行仿真：** [RUN.md](docs/stool_assisted/RUN.md)，统一命令为 `bash roboassemblybench/scripts/run_stool_assisted.sh`。
- **观看已完成结果：** [RESULTS.md](docs/stool_assisted/RESULTS.md)，包含正面、左腕、右腕三路完整录像及完成画面；[本地三视图播放器](docs/stool_assisted/results.html)下载或从本地服务器打开后可播放。
- **查看修复源码、recipe 和工具：** [SOURCE.md](docs/stool_assisted/SOURCE.md)。原成功版本完整源码及工具已随仓库封存，`--prepare` 可校验并展开查看。
- **查看版本与验证范围：** [VALIDATION.md](docs/stool_assisted/VALIDATION.md)、[机器可读交付清单](artifacts/stool_assisted/release.json)及[原运行复盘](artifacts/stool_assisted/round_review.json)。

## 实际装配效果

机器人是两台 UR5e 和两个 Robotiq 2F-85。原任务顺序为 `2 → 3 → 7 → 6 → 4 → 5 → 1 → 0 → 8`。修复后九件均完成安装、释放及对应退避，原运行没有超时或恢复。右臂夹爪与底座的两指典型间隙约为 0.149 / 0.209 mm，持续扶持覆盖全部 166 个左臂阶段；末尾右臂完成开爪。

这是**辅助装配**：抓取达到几何资格后使用实测固定连接，安装后保留世界固定支撑，可理解为虚拟工装。末尾九件在 72 个物理步、约 0.3 秒仿真时间内维持装配状态。该结果没有证明撤除虚拟工装后的自稳、摩擦抓稳或承载能力，也没有证明不同布局下均能完成。

## 交付方式

成功基线的内部追溯编号为 **S24**。后来的 S25 提速试验仅完成 2/9，收益不明显，已回退；S26 离线候选没有混入本次版本。

为保持成功版本和现有 Beam 入口的身份，stool 使用精确冻结的源码及工具，在 `outputs/stool_assisted_reference/` 下展开为独立运行树，资产复用当前仓库中的文件。**通用 `generate_fabrica_canonical_ur5e_demo.sh stool_circular` 和根目录旧 staged recipe 不会自动选择这个修复版本**；请使用上面的专用命令。

本次合并整理不启动新仿真、不消耗原服务器剩余修复轮次。历史完成结果与新部署入口的验证范围分别记录。
