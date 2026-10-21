# 服务器 Beam assisted v35：相对原仓库的改动与交付范围

2026-10-21。本文件说明本次 Draft PR 的代码、功能、来源、验证边界和完整性。原仓库为 `baiyu858/RoboAssemblyBench`，比较基线为 `main@2739e27e66418048d79097376648af1b389a2b68`；提交来自 `HengZhao666/RoboAssemblyBench` 的 `codex/beam-assisted-full-20261021`。

本次整理服务器辅助路线的完整代码和交接，未启动新仿真。服务器 A44 的结果绑定 v35 自己的源/工具/配置身份；不能把它写成“当前上游 main 与本 PR 合并后已经通过完整动态验证”。

## 交付的是什么

本 PR 将服务器当前 889 项源清单的文件覆盖到正常仓库路径，并保留原仓库中不属于该清单的 1758 项路径；同时附服务器源代码和工具的原字节归档，用于独立核对和还原。它是服务器版本的保真交付，尚需维护者审查共享接口及默认值差异后决定如何合并。

相对上述 main，889 项源清单中有 **50 项新增、34 项内容修改、805 项内容相同**。这 84 项源路径的精确差异及上游并行改动交集见 [机器比较清单](../../artifacts/beam_assisted/source_vs_latest_base_analysis.json)。这些数量仅覆盖源清单；额外工具、文档、资产、视频和完整性材料另计，不以此代替整个 PR 的最终文件统计。

交付还包括：

- 服务器实际非缓存工具 **130 项**，其中当前运行封存的关键工具为 **20 项**。不以本机工具目录的额外文件替代服务器工具范围。
- Beam 新增资产 **24 项，约 8 MB**。已与基线相同的 UR5e 资产及配套启动脚本保留现有文件，不重复声称为新增资产。
- 当前短交接、辅助路线检查/发现与账本、共享方法技能、Fabrica 迁移技能、性能分析及复现所需证据索引。
- A44 的前视、左腕、右腕三份完整原始 MP4，保留在 `outputs/beam_assisted_20261015/D28_work/A44_media/episode_0000_live_videos/` 原路径。
- 必要的大 JSON 证据按原字节无损 gzip 保存，并由随附索引/还原工具关联其原路径、字节数与 SHA。历史运行日志和其他历史视频提供服务器文件清单，未将全部历史大产物重复放入 Git。

源/工具归档、发布文件清单、历史产物清单和完整性材料统一从 [artifacts/beam_assisted](../../artifacts/beam_assisted/) 查阅。新建工作副本、上传资料和生成说明不改变运行授权；Beam 暂停状态和旧结论继续保留。

## 辅助路线相对原有流程的改进

上游 Fabrica 流程原本已有 fixed-joint attachment、current-pose capture、锁位及其他配置驱动的辅助路径。当前版本仍属于辅助装配，并非首次发明固定连接，也不证明纯摩擦抓稳、右臂承力或撤除安装支撑后的自然稳定。

主要增量是：限定辅助触发资格，按实测位姿创建连接，保持碰撞；修复完整位姿控制和阶段切换；在最终实际命令后检查完整双臂与持件几何；将安装支撑、开爪、缓慢对齐分成明确生命周期；并保留可核对的原生状态、命令和视频证据。

### 1. 独立任务入口与辅助编译

- 新增 [`beam_assisted_compiler.py`](../../roboassemblybench/core/beam_assisted_compiler.py)，把辅助 Beam 配方编译为明确的抓取、举升、转向、搬运、下降、安装、释放、退避、回位与最终验证阶段。
- 新增 [`fabrica_beam_ur5e_assisted/recipe.yaml`](../../roboassemblybench/tasks/fabrica_beam_ur5e_assisted/recipe.yaml)，在 [`fabrica_canonical.py`](../../roboassemblybench/core/fabrica_canonical.py) 中接入 assisted 分派。既有 task registry 和场景/共享配方继续参与生产构建；`task_registry.py` 与当前 main 相同，不列作本次新增功能。
- 编译时记录对象、阶段、持件关系和辅助合同，分开保留不同路线的机制和成功门。完整服务器树中的历史兼容模块随源清单交付，不将它们的结果混入本次辅助结论。

### 2. 抓取资格、固定连接和安装支撑

- [`beam_assisted_visual_grasp.py`](../../toolkits/factory_dual_franka_assembly/beam_assisted_visual_grasp.py) 依据双指可见表面间隙、交叠情况和连续 native tick 判断接近/闭爪资格，不以“已发闭爪命令”代替实测。
- [`beam_assisted_state.py`](../../toolkits/factory_dual_franka_assembly/beam_assisted_state.py) 在资格成立后，按当时实测 hand 和零件位姿创建 dynamic fixed joint，冻结相对关系。辅助生产路径保留碰撞，不通过直接写零件位姿/速度、切换 kinematic 或关闭碰撞掩盖抓取和运输问题。
- 安装时从实测位置建立世界固定支撑，解除抓取连接；夹爪实测全开后才逐步移动支撑锚点对齐目标。实际物体位姿和运动另行检查，锚点到位不能代替物体到位。
- [`factory_dual_franka_assembly_task.py`](../../internutopia_extension/tasks/factory_dual_franka_assembly_task.py) 将这些状态接入真实 attach、lock、release、观测和成功判定路径，按 assisted 配置选择相应行为。

这种安装支撑主动影响物体运动，最终仍保留世界支撑。Beam 的五件集合、接触表面、目标 frame、阈值和锚点增量依赖本任务，不能直接作为其他 Fabrica 任务的参数。

### 3. 持件控制、阶段切换和最终发布命令

- [`beam_assisted_servo.py`](../../toolkits/factory_dual_franka_assembly/beam_assisted_servo.py)、[`beam_assisted_ik.py`](../../toolkits/factory_dual_franka_assembly/beam_assisted_ik.py) 和 [`plumbers_block_ur5e_skills.py`](../../toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py) 使用冻结抓取变换保持完整 SE(3) 目标，并检查推算姿态与 native 实测之间的残差。
- 修正换段时遗漏横向/旋转目标、转向段多余高度超前、关节姿态分支、末次发布目标保持和停稳判据等问题；不同对象复用可适配的分段搬运逻辑。
- 将请求目标、最终限幅命令、SDK 实际提交和实测执行区分记录；静止臂保留最近有效最终目标，检查原生关节限位。
- [`ur5e.py`](../../internutopia_extension/robots/ur5e.py) 将被 joint override 抑制的 IK 提案单独记录，不再把它算作已提交命令；只有真实 SDK 调用成功后才记入本次提交结果。

### 4. 完整双臂、夹爪、持件和已装件的几何检查

- `beam_assisted_state.py` 的最终 action guard 与随附工具 `visual_mesh_check.py`、`export_geometry.py`、`sweep_model.py`，针对同一最终命令批次核完整双臂、夹爪、持件、已装件及导出环境，包含空手回位。
- [`entity_geometry.py`](../../toolkits/constraint_checking/integration/entity_geometry.py)、[`runtime_monitor.py`](../../toolkits/constraint_checking/integration/runtime_monitor.py) 和 [`stage_precheck.py`](../../toolkits/constraint_checking/integration/stage_precheck.py) 记录实体/唯一 frame 映射、覆盖和缺测；持件仍参与障碍检查，没有新动作的持臂仍占据空间。
- [`contact_policy.py`](../../toolkits/constraint_checking/integration/contact_policy.py) 将允许接触收窄到确切手指与目标对象，避免整段腕掌胶囊被豁免。只读快照避免碰撞分类重复调用带状态变更的抓取资格 getter。
- `local_skills.py` 和 `sequence_precheck.py` 使用任务管理的持件/所有权关系，保留对不相容重复抓取和错误释放的检查；不以辅助生命周期为理由抹掉原报告。

当前精确工具的五分点扫掠仍是有限采样，不是连续 CCD。visible CAD 不等于 PhysX cooked/SDF 碰撞形状，未导出的环境不能声称覆盖。粗胶囊提示、实体观测与有限精确复核各自保留范围。

### 5. 相机、同钟数据、失败尾录和编码

- 新增 [`beam_assisted_camera.py`](../../toolkits/factory_dual_franka_assembly/beam_assisted_camera.py)，绑定真实腕部相机；左右腕光轴各滚转 180°，使录像里夹爪位于画面下方，不依赖后期镜像。
- [`camera.py`](../../internutopia/core/sensor/isaacsim/camera.py)、[`rep_camera.py`](../../internutopia_extension/sensors/rep_camera.py) 与新增 [`beam_video_clock.py`](../../toolkits/factory_dual_franka_assembly/beam_video_clock.py) 读取既有 render-product 的参考时钟，关联三视图、相机参数与 task/epoch/native tick，读取不额外推进物理或渲染。
- [`beam_physics_diagnostics.py`](../../toolkits/factory_dual_franka_assembly/beam_physics_diagnostics.py) 扩充 native 刚体、关节、驱动、实际 SDK 发布及采样边界证据，区分任务 wrapper 与独立 raw 来源，保留读取失败和缺测。
- 新增 [`beam_failure_video.py`](../../toolkits/factory_dual_franka_assembly/beam_failure_video.py)，由 [`generate_demos.py`](../../toolkits/factory_dual_franka_assembly/generate_demos.py) 在失败后调用：锁住首次失败判定和末次发布目标，不再执行 policy，仅继续规定物理尾录。启动前失败或进程崩溃无法制造不存在的尾录。
- `generate_demos.py` 对齐录制 stride 与渲染触发；[`export_lerobot.py`](../../toolkits/factory_dual_franka_assembly/export_lerobot.py) 可复用采集时的完整图像统计，避免编码时重复统计，并保存编码耗时。没有在本次打包中宣称新测得的加速。

### 6. 安全回位和实际框架预检

- 配方追加“左臂 home → 右肩抬离 → 右臂 home”。home 来自实际配置；右肩避让时另外五轴保持入场实测值，已装零件和完整双臂几何继续受检。
- 新增 [`beam_assisted_preflight.py`](../../toolkits/factory_dual_franka_assembly/beam_assisted_preflight.py)，处理真实 `setup_offset_for_assets()` 产生的 `<逻辑名>_<env_id>` 名称映射，拒绝缺失、重复和混淆，修复此前真实构建后因逻辑名/运行时名不同而阻止启动的问题。
- [`test_beam_assisted_preflight.py`](../../tests/toolkits/test_beam_assisted_preflight.py) 包含真实 env0/env1 框架映射和原 A43 失败配置；相关 assisted 控制、释放、相机和失败尾录测试随源清单提交。测试结果与物理运行结果分开解释。

### 7. 工具、交接和其他 Fabrica 任务的迁移

- 附服务器实际使用的 worker、原子 ACK/STOP、从头启动、独立短筛、原件后审、几何导出/检查、录像和完整性工具。130 项实际工具与20项关键封存工具分别列明，不因工具原在仓库外就漏传。
- [`BEAM_REPAIR_HANDOFF.md`](../../BEAM_REPAIR_HANDOFF.md)、[`BEAM_ASSISTED_HANDOFF.md`](../../BEAM_ASSISTED_HANDOFF.md) 与机器注册表将当前状态、源/工具身份、回退、权限及证据路径分开。旧日志、结论和缺测不覆盖。
- [`simulation-repair-loop`](../../skills/simulation-repair-loop/SKILL.md) 将一轮定义为一组可分别验证的无冲突修复加一次正式验收，按差异检查并复用未变域，减少重复后审、专用脚本和文档同步。
- [`fabrica-assisted-repair`](../../skills/fabrica-assisted-repair/SKILL.md)、[`FABRICA_ASSISTED_REPAIR_HANDOFF.md`](../../FABRICA_ASSISTED_REPAIR_HANDOFF.md) 与任务模板说明如何迁移机制，哪些需要新任务适配。它们提供流程和实现映射，尚未完成全部 Beam 模块的通用代码抽取；新任务不能继承 Beam 的参数、运行次数、成功结论或授权。
- [`PERFORMANCE.md`](../../beam_routes/assisted/PERFORMANCE.md) 与机器证据分析已有 59567 步和实际耗时，列出减步、写盘及计算复用候选。这些建议尚不构成已实施提速或动态验证结果。

## 与当前上游并行的共享改动及合并风险

从旧服务器整合点 `6f57dae9f7ab1645ca13facdb9e0f776c81332b7` 到当前 main，上游改动124项路径；其中51项落在本次889源清单中：32项与服务器内容相同，19项双方内容不同。

因此，配置透传、静态环境/材质绑定、部分驱动读回、runner 写入口诊断等已经存在于当前 main 的能力，在完整服务器快照里仍保留，但不能全部记成这次 PR 新增。本文前述新增/修改以机器差异清单为准。

**已经核实的共享默认值变化：**

- `internutopia_extension/configs/robots/ur5e.py`：夹爪默认 `kp/kd` 由当前 main 的 `6e6/8e3` 变为服务器 `7500/173`。
- `internutopia_extension/robots/ur5e.py`：没有显式 pad 摩擦参数时，fallback 由当前 main 的静/动摩擦 `80/70` 变为服务器 `3.0/2.5`。

服务器注释中的“match upstream”对应较早整合基点，不能解释为与当前 main 相同。这些默认值可能影响其他 UR5e 配方。此次为保真交付而保留原字节；转成可直接合并版本时，应审查是否改为 assisted 显式配置、保留共享默认值，并验证受影响任务。没有对这些兼容性问题进行新的仿真或声称它们已解决。

以下19项同时涉及上游并行内容改动，均应审查实际 diff；文件被列入并不等于已证实发生功能回退。未对另一条路线作动态验证或迁移其结果。

- `internutopia_extension/configs/robots/ur5e.py`
- `internutopia_extension/robots/ur5e.py`
- `internutopia_extension/tasks/factory_dual_franka_assembly_task.py`
- `roboassemblybench/core/beam_coordinated.py`
- `roboassemblybench/core/beam_physical_compiler.py`
- `roboassemblybench/core/domain_randomization.py`
- `roboassemblybench/core/fabrica_canonical.py`
- `roboassemblybench/tasks/fabrica_beam_ur5e_staged/recipe.yaml`
- `tests/toolkits/test_beam_canonical_integration.py`
- `tests/toolkits/test_beam_contact_relief.py`
- `tests/toolkits/test_beam_coordinated.py`
- `tests/toolkits/test_beam_idle_hold.py`
- `tests/toolkits/test_beam_physics_contract.py`
- `tests/toolkits/test_beam_physics_diagnostics.py`
- `toolkits/factory_dual_franka_assembly/beam_contact_relief.py`
- `toolkits/factory_dual_franka_assembly/beam_physics_diagnostics.py`
- `toolkits/factory_dual_franka_assembly/beam_support_state.py`
- `toolkits/factory_dual_franka_assembly/demo_policy.py`
- `toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py`

## 已有验证：成功到哪里，缺口在哪里

以下引用原资格和 A44 记录，不是本次 PR 整理重跑的结果。

- v35 真实框架预检受影响检查：**66 passed / 0 failed / 0 skipped**，覆盖 env0/env1 名称映射和原 A43 配置。源889项、关键工具20项与运行封存身份一致，运行后没有相应内容差异。
- A44 为 v35 普通从头运行，完成五件辅助装配、三段回位以及五件/双臂 home 共同240 native tick；`original_success=true`、`assisted_success=true`、`motion_success=true`，主执行后审通过。
- 共59567 task steps；左右六轴最大 home 误差分别为0.0051313592121 / 0.00480947596952 rad，原门为0.01 rad。回位和最终阶段299个存档命令批、1495个独立重放有限几何样本通过，不代表所有控制 tick 均有独立逐步重放。
- 三完整原片各7445帧、30 fps、248.166667秒；服务器完整解码、传输SHA和本机兼容抽检通过。它们与最终独立 raw 的缺测是不同证据。
- **严格独立终态仍缺 raw。** 证书 native tick 为59570；最新独立 after-physics/observation 为59564，末 action/三视图/entity 为59563。240 tick窗口只有60个独立raw索引，其余180不插补。因此 `final_home_review=false`、`accepted_full_assembly=false`、`all_postreview=false`、`formal_success=false`，不能写成全部严格验收通过。
- 原粗胶囊报告仍有 runtime442 / stage376项提示；序列报告仍有10条`release_without_attach`警告、errors0，旧序列模型没有识别辅助生命周期。原报告保留，不能称零告警，也不能直接把胶囊提示解释为精确mesh碰撞。
- 断点恢复没有通过连续/恢复等价验证；旧专项已停止，不能当作可用的从中段复现捷径。

下一轮应优先修终止/采样顺序和合同所需窗口的同钟独立 raw 采集，并与其他可分别验证的修复一起验收；不放宽成功门、不回填历史false。此项在本 PR 中仍为未解决缺口。

## 版本与复核入口

- v35源清单SHA：`6fb47c35bccf5be5fdd854d17c1bfe825dc59d9722708c15510e1217d9fd4fd7`。
- 20关键工具清单SHA：`1b17561c813c4c77b0f5d383d62058c801df7ed64a704beab1e9af8d03af5218`。
- 发布范围、每文件原路径/目标路径/字节数/SHA、归档还原方式与历史大产物范围，查阅随附 `artifacts/beam_assisted/` 清单和索引。
- 当前交接和账本引用原机器绝对路径，以保留历史证据身份；在新机器复现时应依据发布清单还原文件并显式映射工作根，不能假定原服务器绝对路径可直接运行。
- 少量已在889封存清单中的AppleDouble元数据按原件保留，不能当生产Python模块；本机额外缓存或未入权威清单的重复文件不混入服务器身份。

保真归档解决“这次交付来自服务器的哪一版、有没有漏项”的问题；Draft PR 的代码审阅和后续验证解决“这些差异是否适合并入当前 main”的问题。二者分别记录，互不替代。

## 全部源清单内容差异

以下完整列出相对固定基线的34项修改和50项新增，不包含另附工具、交接和证据。`._*`属于清单原有元数据，保真保留，不宣传为新增功能。已有上游文件的执行位保留，服务器原权限保留在原字节包中。

### 修改的34项

- `internutopia/core/sensor/isaacsim/camera.py`
- `internutopia_extension/configs/robots/ur5e.py`
- `internutopia_extension/robots/ur5e.py`
- `internutopia_extension/sensors/rep_camera.py`
- `internutopia_extension/tasks/factory_dual_franka_assembly_task.py`
- `roboassemblybench/core/beam_coordinated.py`
- `roboassemblybench/core/beam_physical_compiler.py`
- `roboassemblybench/core/domain_randomization.py`
- `roboassemblybench/core/fabrica_canonical.py`
- `roboassemblybench/tasks/fabrica_beam_ur5e_staged/recipe.yaml`
- `tests/toolkits/constraint_checking/test_contact_policy.py`
- `tests/toolkits/constraint_checking/test_runtime_monitor.py`
- `tests/toolkits/constraint_checking/test_sequence_precheck.py`
- `tests/toolkits/test_beam_canonical_integration.py`
- `tests/toolkits/test_beam_contact_relief.py`
- `tests/toolkits/test_beam_coordinated.py`
- `tests/toolkits/test_beam_idle_hold.py`
- `tests/toolkits/test_beam_physics_contract.py`
- `tests/toolkits/test_beam_physics_diagnostics.py`
- `tests/toolkits/test_factory_dual_franka_assembly.py`
- `tests/toolkits/test_roboassemblybench_collection.py`
- `tests/toolkits/test_roboassemblybench_position_randomization.py`
- `toolkits/constraint_checking/integration/contact_policy.py`
- `toolkits/constraint_checking/integration/runtime_monitor.py`
- `toolkits/constraint_checking/integration/sequence_precheck.py`
- `toolkits/constraint_checking/integration/stage_precheck.py`
- `toolkits/factory_dual_franka_assembly/beam_contact_relief.py`
- `toolkits/factory_dual_franka_assembly/beam_physics_diagnostics.py`
- `toolkits/factory_dual_franka_assembly/beam_support_state.py`
- `toolkits/factory_dual_franka_assembly/demo_policy.py`
- `toolkits/factory_dual_franka_assembly/export_lerobot.py`
- `toolkits/factory_dual_franka_assembly/generate_demos.py`
- `toolkits/factory_dual_franka_assembly/local_skills.py`
- `toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py`

### 新增的50项

- `RoboAssemblyBench部署进度.md`
- `beam_assembly_changes.md`
- `beam_grasp_handoff.md`
- `internutopia_extension/tasks/._factory_dual_franka_assembly_task.py`
- `roboassemblybench/core/._beam_coordinated.py`
- `roboassemblybench/core/._beam_coordinated_poses.json`
- `roboassemblybench/core/._fabrica_canonical.py`
- `roboassemblybench/core/beam_assisted_compiler.py`
- `roboassemblybench/tasks/fabrica_beam_ur5e_assisted/recipe.yaml`
- `roboassemblybench/tasks/fabrica_beam_ur5e_staged/._recipe.yaml`
- `tests/toolkits/._test_beam_coordinated.py`
- `tests/toolkits/constraint_checking/test_entity_geometry.py`
- `tests/toolkits/test_beam_assisted_camera_contract.py`
- `tests/toolkits/test_beam_assisted_object_servo.py`
- `tests/toolkits/test_beam_assisted_preflight.py`
- `tests/toolkits/test_beam_assisted_release_state.py`
- `tests/toolkits/test_beam_assisted_route.py`
- `tests/toolkits/test_beam_carry_force.py`
- `tests/toolkits/test_beam_close_runtime_fixes.py`
- `tests/toolkits/test_beam_exploration.py`
- `tests/toolkits/test_beam_exploration_integration.py`
- `tests/toolkits/test_beam_failure_video.py`
- `tests/toolkits/test_beam_final_pose_continuation.py`
- `tests/toolkits/test_beam_gripper_closure_range.py`
- `tests/toolkits/test_beam_insertion_completion.py`
- `tests/toolkits/test_beam_loaded_target_hold.py`
- `tests/toolkits/test_beam_pose_stability.py`
- `tests/toolkits/test_beam_release_joint_branch.py`
- `tests/toolkits/test_beam_runtime_integrity.py`
- `tests/toolkits/test_beam_side_clearance.py`
- `tests/toolkits/test_beam_transport_raise.py`
- `tests/toolkits/test_beam_turn_force.py`
- `tests/toolkits/test_beam_turn_speed.py`
- `tests/toolkits/test_beam_unfiltered_contacts.py`
- `tests/toolkits/test_beam_video_clock.py`
- `tests/toolkits/test_beam_weak_side_recenter.py`
- `toolkits/constraint_checking/integration/entity_geometry.py`
- `toolkits/factory_dual_franka_assembly/._beam_support_state.py`
- `toolkits/factory_dual_franka_assembly/._plumbers_block_ur5e_skills.py`
- `toolkits/factory_dual_franka_assembly/beam_assisted_camera.py`
- `toolkits/factory_dual_franka_assembly/beam_assisted_ik.py`
- `toolkits/factory_dual_franka_assembly/beam_assisted_preflight.py`
- `toolkits/factory_dual_franka_assembly/beam_assisted_servo.py`
- `toolkits/factory_dual_franka_assembly/beam_assisted_state.py`
- `toolkits/factory_dual_franka_assembly/beam_assisted_visual_grasp.py`
- `toolkits/factory_dual_franka_assembly/beam_exploration.py`
- `toolkits/factory_dual_franka_assembly/beam_failure_video.py`
- `toolkits/factory_dual_franka_assembly/beam_loaded_target_hold.py`
- `toolkits/factory_dual_franka_assembly/beam_pose_stability.py`
- `toolkits/factory_dual_franka_assembly/beam_video_clock.py`
