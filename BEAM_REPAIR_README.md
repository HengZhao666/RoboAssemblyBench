# Beam 修复说明与阅读导航

这份文档面向第一次查看项目的人：说明 Beam 辅助修复版改了什么、代码从哪里读、录像和结果到哪里找。

**版本范围：Beam assisted v35，代表运行 A44；文档整理日期：2026-10-22。** 本页介绍个人仓库保存的 Beam 交付快照，其他任务的后续修改另行维护。

## 只想快速了解，先看这三处

- **看效果：** [A44 正面完整视频](outputs/beam_assisted_20261015/D28_work/A44_media/episode_0000_live_videos/observation_images_front.mp4)。
- **看结论：** [A44 结果汇总](outputs/beam_assisted_20261015/D28_conclusions.json)，本页下方也有文字解释。
- **看实现：** 先读 [辅助任务配方](roboassemblybench/tasks/fabrica_beam_ur5e_assisted/recipe.yaml)，再读 [阶段编译器](roboassemblybench/core/beam_assisted_compiler.py) 和 [抓取／安装状态实现](toolkits/factory_dual_franka_assembly/beam_assisted_state.py)。

## 相对原版，主要改了什么？

这里比较的是原仓库已有的 Fabrica 辅助装配机制与当前 Beam 辅助实现。原版已经有固定关节、实测位姿捕获、接触判断和锁位等能力；本次工作的增量是把这些辅助用得更受控，并修复实际执行中的问题。

原版机制可对照 [较早上游整合点 6f57dae 的抓取／锁位代码](https://github.com/baiyu858/RoboAssemblyBench/blob/6f57dae9f7ab1645ca13facdb9e0f776c81332b7/internutopia_extension/tasks/factory_dual_franka_assembly_task.py#L3726)。这提供明确的原代码核对点；该提交日期为 2026-10-10。

### 1. 抓取辅助保留，但更明确地检查“什么时候可以固定”

原版已有固定连接，也有接触检查。本版进一步检查两侧手指与零件可见表面的间隙、是否交叠以及是否稳定，合格后才按当时实测位置建立夹爪—零件固定连接，并保存抓取时的相对位置和姿态。

搬运中继续检查这个相对关系与仿真实测的误差。闭爪命令发出，不等于已经抓好；固定连接建立，也不证明零件仅靠摩擦就能抓稳。

### 2. 安装辅助从直接定位，改成约束接住后缓慢对齐

原版代码有直接写零件位姿、清速度、按配置关闭碰撞以及吸附／锁定等路径。本版辅助生产路径保留零件碰撞，抓取和安装阶段不靠这些直接写入来消除误差。

安装时，先在零件实测位置建立世界固定关节作为支撑；夹爪实测完全张开后，再小步移动约束锚点，使零件缓慢对齐安装目标。程序还会核对零件实际位置，不能只凭锚点到位就算装好。

**最后仍保留安装支撑。** 因此这是辅助装配结果，不能据此证明撤掉支撑后组件仍自然稳定。

### 3. 修复搬运和阶段切换中的控制问题

针对第四件、第五件等阶段暴露的问题，修正了换段时位置／姿态目标不连续、转向时多余高度、手腕姿态分支、停稳误判以及最终目标保持等问题。控制器区分“计划目标”“最终下发命令”和“实际执行结果”，并检查手腕推算的零件位置与实测位置是否一致。

第五件在另一条机械臂附近的搬运，也纳入分段避让和双臂几何检查。

### 4. 扩展双臂、夹爪和零件的检查范围

检查对象包括两条机械臂、完整夹爪、正在搬运的零件及已经安装的零件；静止的另一条机械臂也占据空间。检查针对最终限幅后的实际命令，覆盖搬运与空手回位。

已有独立几何复核采用有限采样，不能等同于每个物理时刻都完成了连续碰撞证明。原报告中的提示、警告和缺测均保留。

### 5. 补齐相机、回位与真实启动检查

左右腕相机绕光轴旋转 180°，使视频里夹爪位于画面下方。装配后追加“左臂回初始位 → 右肩抬离避让 → 右臂回初始位”，并核对实际关节位置。

同时修复真实框架构建后逻辑名称与运行时名称不一致的问题，增加同一物理时刻的三视图、命令和状态记录，便于定位故障。

## 查看代码，建议按什么顺序？

下列链接都指向本仓库实际文件。历史目录名中的 franka 不代表本任务使用 Franka；这里的辅助配方实际使用双 UR5e 与 Robotiq 夹爪。

1. **任务入口与完整动作顺序。**  
   [recipe.yaml](roboassemblybench/tasks/fabrica_beam_ur5e_assisted/recipe.yaml) 选择辅助路线；[beam_assisted_compiler.py](roboassemblybench/core/beam_assisted_compiler.py) 生成抓取、举升、转向、搬运、下降、释放、退避、回位等阶段；[fabrica_canonical.py](roboassemblybench/core/fabrica_canonical.py) 接入辅助分派。

2. **固定连接、安装支撑、释放和最终判定。**  
   重点看 [beam_assisted_state.py](toolkits/factory_dual_franka_assembly/beam_assisted_state.py)。可搜索 _assisted_attach、_assisted_lock、_assisted_update_install_alignment；抓取资格的几何计算在 [beam_assisted_visual_grasp.py](toolkits/factory_dual_franka_assembly/beam_assisted_visual_grasp.py)。

3. **零件如何跟着夹爪运动，以及误差如何处理。**  
   看 [beam_assisted_servo.py](toolkits/factory_dual_franka_assembly/beam_assisted_servo.py) 和 [beam_assisted_ik.py](toolkits/factory_dual_franka_assembly/beam_assisted_ik.py)。与动作技能的衔接在 [plumbers_block_ur5e_skills.py](toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py)。

4. **这些逻辑怎样进入真实任务、怎样下发给机械臂。**  
   看 [factory_dual_franka_assembly_task.py](internutopia_extension/tasks/factory_dual_franka_assembly_task.py) 的辅助状态接入，以及 [ur5e.py](internutopia_extension/robots/ur5e.py) 的实际命令提交记录。

5. **碰撞和几何检查。**  
   从 beam_assisted_state.py 的最终动作检查读起，再看 [实体几何](toolkits/constraint_checking/integration/entity_geometry.py)、[运行时检查](toolkits/constraint_checking/integration/runtime_monitor.py)、[接触允许范围](toolkits/constraint_checking/integration/contact_policy.py)。独立几何复核工具在 [visual_mesh_check.py](outputs/beam_assisted_20261015/tools/visual_mesh_check.py) 和 [sweep_model.py](outputs/beam_assisted_20261015/tools/sweep_model.py)。

6. **相机、启动和采集。**  
   看 [腕部相机](toolkits/factory_dual_franka_assembly/beam_assisted_camera.py)、[真实配置预检](toolkits/factory_dual_franka_assembly/beam_assisted_preflight.py)、[采集主入口](toolkits/factory_dual_franka_assembly/generate_demos.py)。运行与后审工具集中在 [tools 目录](outputs/beam_assisted_20261015/tools/)。

按问题查测试，可看 [辅助路线](tests/toolkits/test_beam_assisted_route.py)、[持件控制](tests/toolkits/test_beam_assisted_object_servo.py)、[释放状态](tests/toolkits/test_beam_assisted_release_state.py) 和 [真实配置预检](tests/toolkits/test_beam_assisted_preflight.py)。这些测试验证相应代码行为，完整装配结果另看实际运行证据。

## 查看结果，应该去哪看？

### 三路完整视频

三份录像来自同一轮 A44，各约 4 分 8 秒、30 fps，保留原片，没有为上传重新剪辑或编码。

- [正面视频：查看整体装配和双臂回位](outputs/beam_assisted_20261015/D28_work/A44_media/episode_0000_live_videos/observation_images_front.mp4)
- [左腕视频：查看主要抓取、搬运和插入](outputs/beam_assisted_20261015/D28_work/A44_media/episode_0000_live_videos/observation_images_left_wrist.mp4)
- [右腕视频：查看右臂附近状态](outputs/beam_assisted_20261015/D28_work/A44_media/episode_0000_live_videos/observation_images_right_wrist.mp4)

如果 GitHub 页面不能直接播放，可使用页面上的下载或 View raw 获取原始 MP4。

### A44 实际达到了什么？

A44 在服务器 v35 上从头完成五件辅助装配、三段双臂回位，并通过原任务的共同稳定窗口判定。主程序记录 original_success、assisted_success、motion_success 为 true，主执行证据检查通过。

**严格独立验收仍未全部通过。** 原因是末尾独立原始状态采样没有覆盖最终判定时刻和完整窗口，因此 accepted_full_assembly、final_home_review、formal_success 仍为 false。视频展示的动作完成与完整独立证据验收，需要分别理解；后者的缺口没有用视频或旧记录补写为通过。

要核对原件，按此顺序看：

1. [D28_conclusions.json：本轮结果、版本、回位误差与缺口总览](outputs/beam_assisted_20261015/D28_conclusions.json)。
2. [execution_evidence.json：主执行证据检查](outputs/beam_assisted_20261015/D28_work/A44_media/execution_evidence.json)。
3. [final_home_independent_read_review.json：独立终态验收为何未通过](outputs/beam_assisted_20261015/D28_work/final_home_independent_read_review.json)。
4. [D28_local_media_review.json：视频完整性、传输校验和抽查记录](outputs/beam_assisted_20261015/D28_work/D28_local_media_review.json)。

进一步查看原报告、采样日志和压缩证据，可从 [证据索引](artifacts/beam_assisted/evidence_index.json) 查具体文件。部分大 JSON 以 .json.gz 无损保存；[原件还原说明](BEAM_ASSISTED_PR_HANDOFF.md) 和 [校验／还原工具](scripts/beam_assisted/verify_publication.py) 说明如何读取。

## 需要精确比较文件、复现或继续开发？

- **全部文件与功能差异：** [CHANGES_FROM_UPSTREAM.md](docs/beam_assisted/CHANGES_FROM_UPSTREAM.md) 和 [机器比较清单](artifacts/beam_assisted/source_vs_latest_base_analysis.json)。其中“50 项新增、34 项修改”针对固定上游基线 2739e27，不是相对所有历史原版的累计统计。
- **环境、资产和兼容性：** [DEPLOYMENT.md](docs/beam_assisted/DEPLOYMENT.md)。共享 UR5e 默认驱动／摩擦配置与该上游基线有差异，部分工具仍依赖原服务器布局和外部 Isaac Sim／Conda 环境，不能视为任意机器的一键运行包。
- **服务器原始源码和工具：** [完整归档](artifacts/beam_assisted/server_code_and_tools.tar.gz)、[归档清单](artifacts/beam_assisted/package_manifest.json) 与 [原交付逐文件清单](artifacts/beam_assisted/publication_manifest.json)。原清单对应既有交付快照；后来增加的本页由新的 Git 提交保存，不改写旧封存身份。
- **继续 Beam：** 从 [BEAM_REPAIR_HANDOFF.md](BEAM_REPAIR_HANDOFF.md) 进入辅助路线，核对交接及本任务账本；本页不提供新的仿真授权。
- **复用到 stool 等其他任务：** 从 [FABRICA_ASSISTED_REPAIR_HANDOFF.md](FABRICA_ASSISTED_REPAIR_HANDOFF.md) 和 [迁移技能](skills/fabrica-assisted-repair/SKILL.md) 开始。当前 Beam 辅助编译器仅支持 Beam，需要为新任务适配几何、抓取和装配顺序。
