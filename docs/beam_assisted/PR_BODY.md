本 PR 将服务器当前 **Beam assisted v35 完整代码、工具、交接和最新完整视频**整理为可审查交付。比较基线为 `baiyu858/RoboAssemblyBench main@2739e27e66418048d79097376648af1b389a2b68`，分支为 `HengZhao666:codex/beam-assisted-full-20261021`。

## 代码与功能改动

- 新增独立 assisted recipe/compiler，将抓取、分段搬运、安装支撑、释放、双臂回位和最终共同稳定验证连接到真实生产链。
- 在双指实测几何资格和稳定条件成立后创建固定连接，冻结抓取关系、保留碰撞；安装时按实测位置建立世界支撑，开爪后逐步对齐。上游已有固定连接等辅助，本 PR 主要补资格、生命周期和取证，**仍是辅助装配**，不代表自然摩擦抓稳或撤除支撑后稳定。
- 修复持件完整位姿保持、阶段切换目标、转向高度超前、原生关节分支、停稳判断及最终实际发布目标保持；针对最终 SDK 命令检查完整双臂、夹爪、持件和已装件几何。
- 增加同 native tick 的命令/刚体/三视图证据、原生腕部相机和失败尾录；两腕光轴滚转180°，使夹爪在画面下方。追加左home→右肩抬离→右home，修复真实框架逻辑名/运行时名映射预检。
- 附差异验证流程、Fabrica辅助迁移技能、短交接、原结论和性能分析；迁移仍需任务专属资产、frame、路径、阈值及独立授权。

详细到文件的说明、19项上游并行改动列表及限制见 [CHANGES_FROM_UPSTREAM.md](https://github.com/HengZhao666/RoboAssemblyBench/blob/codex/beam-assisted-full-20261021/docs/beam_assisted/CHANGES_FROM_UPSTREAM.md)。

## 完整性与共享影响

889项源清单覆盖到正常仓库路径，相对基线为 **50新增、34修改、805相同**；保留上游清单之外1758项路径。另附服务器原字节代码/工具归档、实际130项非缓存工具（20项关键工具单独封存）、24项新增运行资产及1个目录标记、交接及逐文件清单。必要大JSON以gzip无损保存，随附索引/还原工具；历史日志和其他历史视频保留服务器清单，不全部重复入Git。

A44前视、左腕、右腕三份完整原MP4按用户要求加入Git，原路径为 `outputs/beam_assisted_20261015/D28_work/A44_media/episode_0000_live_videos/`。资产和证据处理边界均在随附清单中明确，不将未上传的历史大产物写成已上传。

从旧整合点到当前main的上游改动与源清单交集51项：32项已相同，**19项双方内容不同**。其中共享默认值确有变化：UR5e夹爪kp/kd从当前main的`6e6/8e3`变为服务器`7500/173`，默认pad摩擦从`80/70`变为`3.0/2.5`。本次保留服务器原字节，可能影响其他UR5e配方，须审查隔离配置与兼容性；不能称所有修改只影响Beam。完整性材料和精确84项源差异见 [artifacts/beam_assisted](https://github.com/HengZhao666/RoboAssemblyBench/tree/codex/beam-assisted-full-20261021/artifacts/beam_assisted)。

## 已有验证与未解决缺口

- v35受影响预检 **66 passed / 0 failed / 0 skipped**，包含真实env0/env1框架映射和原失败配置。
- 已有A44从头运行完成五件辅助装配、三段回位及共同240 native tick：`original_success=true`、`assisted_success=true`、`motion_success=true`；主执行后审通过。59567步，左右最大home误差约0.00513/0.00481 rad；回位/最终段299存档命令批、1495有限几何样本复核通过。
- 三原片各7445帧、30fps、248.166667秒，已有完整解码、传输SHA及本机兼容抽检记录。
- **严格终态raw缺测尚未修复**：证书tick59570，最新独立raw早6tick、末action/三视图/entity早7tick；240窗口仅60个独立raw索引。因此 `final_home_review=false`、`accepted_full_assembly=false`、`all_postreview=false`、`formal_success=false`，不将主成功写成全部严格验收通过。
- 原胶囊与序列警告、有限采样/未覆盖项及断点恢复未验证状态保留。有限扫掠不是连续CCD，fixed joint和世界支撑不证明自然抓稳。

本次整理未启动新仿真。A44验证绑定服务器v35身份，**不证明最新main与本PR组合后的动态兼容性**。保持Draft，用于完整代码交付、共享默认值/并行差异审查和后续修复；下一步优先补终止同钟独立raw采集，不放宽成功门、不回填旧结论。

## 完整原视频与接手入口

[前视原片](https://github.com/HengZhao666/RoboAssemblyBench/blob/codex/beam-assisted-full-20261021/outputs/beam_assisted_20261015/D28_work/A44_media/episode_0000_live_videos/observation_images_front.mp4) / [左腕原片](https://github.com/HengZhao666/RoboAssemblyBench/blob/codex/beam-assisted-full-20261021/outputs/beam_assisted_20261015/D28_work/A44_media/episode_0000_live_videos/observation_images_left_wrist.mp4) / [右腕原片](https://github.com/HengZhao666/RoboAssemblyBench/blob/codex/beam-assisted-full-20261021/outputs/beam_assisted_20261015/D28_work/A44_media/episode_0000_live_videos/observation_images_right_wrist.mp4)

[PR交接入口与校验/还原方法](https://github.com/HengZhao666/RoboAssemblyBench/blob/codex/beam-assisted-full-20261021/BEAM_ASSISTED_PR_HANDOFF.md)。
