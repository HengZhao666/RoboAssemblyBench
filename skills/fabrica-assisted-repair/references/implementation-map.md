# Fabrica 辅助装配迁移：实现映射与适用边界

2026-10-21。范围：分析和文档；没有修改源码、部署、创建任务实例或启动仿真。尚未选择下一项 Fabrica 任务，本文不替用户选择，也不提供执行授权。

本GitHub发布副本使用仓库相对源码链接；服务器原文保留在完整快照中。部署位置仍以当前任务交接为准，不能把本文链接当新任务权威身份。

## 1. 基线和验证边界

本文映射基于 Beam assisted v35 已核验副本；本次逐 SHA 核验本机889源/20关键工具与清单相符。权威路径、源/工具清单与回退只在[Beam 当前交接](../../../BEAM_ASSISTED_HANDOFF.md)维护；新任务必须核自己的身份，不能把本映射当作当前服务器已核验的证明。

A44 主任务及回位成功，但严格独立终态 raw/稳定窗口存在缺测，`accepted_full_assembly=false`。细节只读 [D28 原结论](../../../outputs/beam_assisted_20261015/D28_conclusions.json)；此缺口尚未修复。固定连接与安装世界支撑属于辅助，不能证明摩擦自然抓稳、右臂承力或撤除支撑后的稳定。

## 2. 先认清现有平台，再谈 Beam 迁移

现有 generic canonical 从头装配流程本来就有物理辅助。它已经生成 `attachment_mode: fixed_joint`、`attachment_relative_pose_source: current`、接触资格与释放/锁位阶段。固定关节及“按当前相对位姿连接”不是 Beam 首创。

具体入口：

- 任务入口解析：[task_registry.py:351](../../../roboassemblybench/core/task_registry.py#L351) 的 `load_task_recipe()`，负责 recipe/extends、场景 profile、annotation、metadata 及指纹。
- 任务清单与配方位置：同文件 `list_task_recipes()` / `resolve_task_spec_path()`；配方在 `roboassemblybench/tasks/<任务标识>/recipe.yaml`。canonical 资产/零件/装配步骤来自 [canonical_tasks.json](../../../roboassemblybench/assets/Fabrica/canonical_7_bundles/canonical_tasks.json)，当前 schema 为 v3、含 7 个 canonical 装配定义；这里只定位 catalog，不把任何一项选作新任务。
- 通用编译：[fabrica_canonical.py:6034](../../../roboassemblybench/core/fabrica_canonical.py#L6034) 的 `compile_fabrica_canonical_recipe()`、同文件 `_compile_targets_and_phases()` / `_append_pick_and_attach_phases()` / `_append_release_and_retreat()`。
- 现有辅助与可选干预：`_append_pick_and_attach_phases()` 已设置 current-pose fixed joint；`_append_release_and_retreat()` 已有 freeze-current lock。执行层 [factory_dual_franka_assembly_task.py:4593](../../../internutopia_extension/tasks/factory_dual_franka_assembly_task.py#L4593) 的 `_attach_object()` 和同文件 `_lock_object()` 还支持配置驱动的 snap/local-offset、位姿固定、清速度、碰撞过滤等路径。不同 recipe 启用的选项不同，不能笼统说所有原版任务都关闭碰撞或都靠瞬移。
- 配方生成后的实际任务模型：[scene_builder.py:527](../../../toolkits/factory_dual_franka_assembly/scene_builder.py#L527) 的 `build_dual_franka_assembly_episode()`；任务执行器为 `FactoryDualFrankaAssemblyTask`。

Beam 相对已有平台的主要改进：将辅助限制为通过实测资格后创建的物理约束，保持碰撞，禁止生产中直接重写 Beam 刚体位姿/速度；明确冻结抓取关系与 native 漂移保护；修正换段目标、停稳测量、关节分支及最终发布后的几何检查；用实测位置接住、张开后缓慢对齐的支撑生命周期代替直接摆正。新任务应先审原 recipe/执行分支，保留已存在且适用的机制，再适配这些增量。

## 3. 已有代码机制如何接到新任务

### 抓取资格与实测固定连接

入口为 [beam_assisted_visual_grasp.py:122](../../../toolkits/factory_dual_franka_assembly/beam_assisted_visual_grasp.py#L122) 的 `close_action()` / `ready()` / `native_gaps()`，随后 [beam_assisted_state.py:177](../../../toolkits/factory_dual_franka_assembly/beam_assisted_state.py#L177) 的 `_assisted_attach()` / `_assisted_joint()`。任务 `_attach_ready()` / `_attach_object()` 通过 opt-in 字段调入。

可迁移的原则是“实测双指几何资格成立并稳定后，读取刚体和手腕姿态，创建并冻结 fixed joint”。Beam 当前资格是两侧可见 pad 间隙 5–250μm、无交叠和 12 个物理时钟 tick；这是辅助几何资格，不能改称测到了夹持力。

**必须分别标明手腕、hand 和控制 TCP。** 物理关节通过 native hand/body 实测位姿计算连接锚点；控制附件还保存相对 task TCP 的变换。迁移时应显式保存 `T_world_hand`、`T_hand_object`、`T_hand_tcp` 与 `T_tcp_object` 的关系、四元数顺序、单位、body 路径及 joint 锚点读回。不能把一个名为 `T_wrist_object` 的字段同时当作不同 frame 的变换，也不能每 tick 重锚把漂移抹掉。

这部分当前仍绑定 Robotiq 指尖网格名、关节布局、驱动 0–0.8rad 及 openness 归一化。别的夹爪、零件曲面/薄壁、接触允许区需要几何适配器与重新验证。

### 保持碰撞和 native 漂移保护

`_assisted_joint()` 明确开启 joint collision；`_assisted_attach()` 保存 `collision_disabled=False`。任务执行层 `_set_object_collision()`、`_set_object_pose()`、`_zero_object_velocity()` 等入口对 Beam assisted 的不允许操作设防。其前缀/路线识别需要适配新任务，不能只依赖重命名文件。

控制链位于 [plumbers_block_ur5e_skills.py:2267](../../../toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py#L2267) 的 `_assisted_object_servo_context()` / `_assisted_check_model_residual()` / `_assisted_verify_published_held_command()`，以及 [beam_assisted_servo.py:42](../../../toolkits/factory_dual_franka_assembly/beam_assisted_servo.py#L42) 的 `object_pose_from_tcp()`、`anchored_object_prediction()`、`validate_model_native_residual()`、`descent_admission()`。

可以复用 SE(3) 组合、身份/epoch 缓存、原生时钟连续资格、COM 位姿差分停稳及“保留完整最终发布状态”的机制。Beam 的 100μm/1.5mrad 模型门、150μm/1mrad 插入门、12tick、横向积分/冷轴/速度等数值和相应 frame 要由新任务合同重新确定，不能默认复制，也不能遇到不通过就放宽。

### 最终 SDK 指令、双臂和全几何保护

最终入口为 [beam_assisted_state.py:754](../../../toolkits/factory_dual_franka_assembly/beam_assisted_state.py#L754) 的 `_assisted_guard_actions()`：实际关节 → 最终限幅/原生限位 → 同一完整双臂批次的几何检查 → 返回 SDK 批次。不能把早期请求或理想 IK 路径的检查当作最后实际发布目标的检查。

工具侧入口为 [assisted_worker.py:328](../../../outputs/beam_assisted_20261015/tools/assisted_worker.py#L328) 注入的 `production_cad_guard()` 与 [visual_mesh_check.py:77](../../../outputs/beam_assisted_20261015/tools/visual_mesh_check.py#L77) 的 `ProductionCarryChecker.check()` / `check_empty()`；`export_geometry.py` 和 `sweep_model.py` 提供本轮资产/关节/刚体对应。

可复用的是完整双臂、完整夹爪、持件、全部已装件和相关导出环境同时参与；同钟实测校准预测增量；区分目标接触与非目标碰撞；空手也检查完整双臂。必须换成新任务实际资产拓扑与接触白名单。目前工具强制 `54` 网格，空手模式强制五个 `fabrica_beam_*` 对象；这不是通用接口。五分点扫掠属于有限采样，不是连续 CCD，visible CAD 不等于 PhysX cooked/SDF，未导出 warehouse 几何也不能宣布覆盖。零件数量与检查域应来自任务清单，不能简单删掉 54/五件断言后宣称通用。

### 安装世界支撑、张开和缓慢对齐

入口为 [beam_assisted_state.py:238](../../../toolkits/factory_dual_franka_assembly/beam_assisted_state.py#L238) 的 `_assisted_lock()`、`_assisted_update_install_alignment()`、`_assisted_release_open()`、`_assisted_installation_sample()`。

机制可迁移：从当时实测零件位姿建立世界 fixed support，解除抓取连接；夹爪实测张开后逐步移动约束锚点到目标；实际零件姿态和 native 运动另行验收，不按“锚点已到位”直接判零件到位。它没有直接写刚体位姿/速度，但主动移动世界约束仍然是有意的物理辅助，不能描述成自然落位。

Beam 的上方 3mm、每物理步≤0.1mm/0.2mrad、`_assembled` 目标命名、世界参考、允许世界锁定的对象都不能盲搬。新任务需要核对装配步骤中的 `move_part`、`socket_part`、`optimizer_hold_part`、抓取与 disassembly path，对象相对哪一个活动/静止父件装配；先把目标 frame 转换为本次实际世界位姿。若父件还会运动，世界锁定可能破坏任务运动学，必须明确任务专属支撑策略。

### 双臂协作和安全回位

Beam 配方层为 [beam_assisted_compiler.py:12](../../../roboassemblybench/core/beam_assisted_compiler.py#L12) 的 `_add_visual_base_support()` / `_append_final_home_return()`；运行门为 `_assisted_home_sample()`、`consecutive_native_gate()`、`_assisted_final_success_ready()`；原生角度移动技能在 `UR5eAssemblyAtomicSkillAdapter._move_arm_to_joint_positions_action()`。

可以复用双臂所有权检查、静止臂保留最近有效最终目标、原生关节分支比较、实际到位而不是计时到位、无持件/实测开爪检查、任务结束后安全返回的要求。Beam 的“右臂近夹底梁、左臂装配”只证明视觉保持，没有右臂承力证明；三段回位顺序、右肩 -0.24rad、六轴布局、0.01rad/8tick 均依赖当前资产与路径。新任务从实际终态重新规划/筛查，不能下发 Beam 或 A42 的旧关节常量。

真实框架映射不能省：`setup_offset_for_assets()` 将配置名改为 `<逻辑名>_<env_id>`，但阶段引用继续使用逻辑名。v35 的 `_home_robot_configs()` 在 preflight 中做唯一映射并拒绝缺失/重复/混淆；原测试用真实 env0/env1 映射和 A43 原配置作反例。迁移时应复用实际框架路径，覆盖模型子类/接口注册/metadata 覆盖边，不能只做字符串替换的假测试。

### 三视图、同钟数据、尾录与最终证书

相机入口为 [beam_assisted_camera.py:71](../../../toolkits/factory_dual_franka_assembly/beam_assisted_camera.py#L71) 的 `NativeWristCameras`；启动渲染绑定/只读不变性检查在 `assisted_worker.py` 的 `review_initial_images()` / `camera_render_parameter_comparison()`。RGB、深度与 camera params 应绑定同一个真实 render product；腕部 180°是实际相机光轴滚转，不是后期镜像。相机位置、安装 body、可见 pad、裁剪范围和朝向对新资产需适配，不照搬左右不同 offset。

原生采集入口为 [beam_physics_diagnostics.py:452](../../../toolkits/factory_dual_franka_assembly/beam_physics_diagnostics.py#L452) 和 `internutopia/core/runner.py` 的 before-action / after-action / after-physics / after-observation 采样。任务/epoch/native index/time、body/view 身份、完整双臂关节/驱动目标、实际刚体及三视图应对齐。独立 raw 与 entity exact-wrapper 数据是不同观测源，不能用 44 个 entity 位姿称独立 raw 已覆盖全部 44 个实体。

尾录入口为 [beam_failure_video.py:26](../../../toolkits/factory_dual_franka_assembly/beam_failure_video.py#L26) 的 `record_failure_tail()`，由 `generate_demos.py` 在失败后调用：锁住失败判定、保留最后发布目标、不再运行 policy，仅继续约定 5 秒物理观察和原视频；原生崩溃/启动前失败无法制造尾录。该模块 `_is_beam()`、三相机数量等仍有任务识别硬编码，新任务必须显式适配；A44 成功轮没有重新测试失败尾录，不能把它列成本轮新增通过项。

最终证书机制是“原生身份、目标/对象/开爪状态和连续共同稳定窗口成立，再同钟复查”，入口为 `BeamAssistedState._assisted_conditions_stable()` / `_assisted_final_success_ready()` 和任务 `calculate_metrics()`。原始报告与独立后审分别保存。迁移时优先补完整终止采集边：终止前/关闭 handle 前只读采集证书对应当前 native tick，不额外推进物理、不给新动作；若合同要求独立验证整个稳定窗口，还需覆盖每个所需 tick，不能用 stride 采样插补。这个能力在 Beam A44 尚有缺口，当前只是明确的改进目标。

## 4. 哪些不能整文件复制

以下是当前源码约束，不是已抽取的跨任务组件：

- `compile_assisted_beam_recipe()` 明确拒绝非 Beam；`compile_fabrica_canonical_recipe()` 中的 assisted 分派直接进入它。给其他任务添加 `beam_execution_route: assisted` 并不会自动获得通用版本。
- `fabrica_beam_6/3/2/1/0`、固定五件集合、phase 名/后缀、安装累计顺序和 `_assembled` 世界目标；不能用于任意对象数量/装配父 frame。
- 只对第四件的 solver position iterations64；不复制给其他任务，也不将 USD authored 迭代数称为已读回的 native island 有效迭代数。
- 双侧 pad 模型、Robotiq 归一化、UR5e 六轴顺序/限位和 native body 名；其他机器人/夹爪需独立接口适配。
- 举升/转向/平移/下降路径、入口负载偏移保持、第五件避右臂走廊、右扶位置和最终右肩抬离量；新抓取关系与终态变化会使旧路径证据失效。
- full54、固定五个 installed objects、目标接触豁免、静态支撑接口及环境 mesh 图；新任务需重新导出并检查资产实例/拓扑/碰撞对应。
- Beam 特定误差、速度、超前、插入、开爪、原生 tick 和最终成功数值；新任务应有独立且事先确定的合同。
- 录制/原生采集对 Beam 的识别、三视图命名、旧授权 D/A 编号及账本路径；不把 Beam 次数、历史结果或原视频纳入新任务完成证明。

## 5. 新任务最小落地路线（尚未执行）

1. 用户选定任务后，先从 catalog → recipe → canonical compiler → 实际配置 → 运行入口审原实现，列出原有辅助、目标父 frame、零件与夹爪资产、完成条件和差异。记录从头可运行基线；没有基线结果就明确未知，不先重写平台。
2. 用任务专属 assisted adapter 提供对象/阶段/接触对/支撑资格/目标 frame/终态合同；保留同一原平台构建与执行链。先复用已存在的通用模块，再确定哪些 Beam 局部实现值得拆成公共模块。本文没有完成公共代码抽取。
3. 新任务先选自己的工作副本及源/工具/资产/配置身份，沿用或建立本任务账本与成功合同，不改 Beam 权威 A，不占用 Beam 账本或默认授权。辅助假设与副作用必须明确：何时允许持件固定、哪些对象能获世界支撑、最终支撑是否保留。
4. 先用真实原件做负例与短正例、真实框架映射、最终 SDK 指令和几何检查；新增阶段只扩大受影响范围，能分别验证的改动合并。是否做短物理、完整运行和上限由新任务授权决定；配置检查、运动学代理、exit0 与原任务历史成功均不算该任务动态成功。
5. 成功合同同时要求实际抓取/运动/释放/共同稳定条件与完整证据。先补终止同钟独立原生采集的测试，避免把 A44 的缺测带到新任务。完整原片逐 SHA 下载、只做一次完整解码，机器后审一次；人工聚焦改动段、关键切换和首失败。

