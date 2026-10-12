# Beam 连续物理修复：代码与实验审查说明

> **供原作者审查的进行中版本，不是完整装配成功版本。**
> 本次整理于 2026-10-09，以原仓库 `main` 提交 `4b5a89410c5cd571d218d58da5f65a16682a1a23` 为基线。
> 当前保留状态：连续接触抓持、原生承托台、真实承重检查、固定空闲臂命令及诊断能力；PGS 和降低机械臂驱动增益的试验已回退。
> 完整五零件装配、插入、换扶与最终松手尚未通过验证。

## 先看什么

1. 看本页了解改动位置、证据边界和公共代码影响。
2. 看[关键实验索引](../../outputs/beam_repair_review/README.md)，每个节点都有正面、右腕视频及对应日志。
3. 看[当前源码 SHA-256 清单](current_source_manifest.json)和[最终保留状态](../../outputs/beam_repair_review/final_retained/final_verification.json)。
4. 重点审查：抓持登记有没有改写自由零件状态；指垫材料是否绑定实际碰撞体；控制交接是否连续；落桌门槛是否依赖可信物理读数；公共分支是否意外影响其他任务。

## 为什么修复

原始 canonical 装配生成器使用 `fixed_joint` 搬运，并安排初始零件锁定、锁定时关闭碰撞、释放后冻结位姿。公共执行层支持脚本写位姿和清零速度。这些机制可以用于流程演示，但会给待装零件提供实际夹爪、桌面和接口以外的支撑，不能由其成功结果证明连续接触装配已经成立。

固定工作台、机器人安装座和承托台本身，是保留的工装建模假设；本次要取消的是自由待装零件的辅助搬运与停稳。没有尝试同时修复其他六个 canonical 任务的完整动作流程。公共对象构造和元数据变更仍可能影响它们，需要回归验证。

## 改动在哪：从现象找到修改入口

### 任务生成、参数和装配顺序

- [`fabrica_canonical.py`](../../roboassemblybench/core/fabrica_canonical.py)：为 Beam 注入 `official_bimanual_hold`、`continuous_physics` 和连续控制策略；修正抓取变换、支撑碰撞体及参数传递。连续模式在编译阶段限制为 Beam；其他 canonical 任务仍保留原默认路径。
- [`Beam recipe.yaml`](../../roboassemblybench/tasks/fabrica_beam_ur5e_staged/recipe.yaml)：策略开关和任务参数入口。当前机械臂增益恢复到 `kp=80000/kd=4000`，指垫摩擦试验值为 `1.0/0.8`，不是实机标定值。
- [`beam_coordinated.py`](../../roboassemblybench/core/beam_coordinated.py) 与 [`beam_coordinated_poses.json`](../../roboassemblybench/core/beam_coordinated_poses.json)：落桌、扶持、换扶、双手交接与最终释放的阶段规划。几何候选已经写入代码，后续阶段尚未在完整受力仿真中验证。

出现“装配顺序不合理、目标放错、阶段缺失、参数未送入执行层”时，从这一组文件查起。

### 接触、抓持登记和成功判断

- [`factory_dual_franka_assembly_task.py`](../../internutopia_extension/tasks/factory_dual_franka_assembly_task.py)：读取手指与指定目标的配对接触力，要求双侧接触与合适几何；连续抓持登记只保存观察状态，避免建立额外连接、重新写位姿、清零速度或重置滑移参考。增加共享抓持状态和失夹后的阶段保护。
- [`beam_support_state.py`](../../toolkits/factory_dual_franka_assembly/beam_support_state.py)：从有效物理视图复制世界位姿和六维速度，读取承重面接触力及间隙，并独立累计落桌稳定性。旧速度接口只用于诊断。

落桌仍要求承重至少 `1.962 N`、底面间隙 `[-0.5, 1] mm`、线速度不超过 `2 mm/s`、角速度不超过 `0.03 rad/s`，连续 `96` 步成立。**没有用画面静止或很小的首尾位移替代该验收。**

出现“空夹却通过、失夹仍继续、零件突然停住、看似落桌但没有承重、成功误判”时，查这一组文件及生成后的阶段配置。

### 动作执行、微调与落桌受力

- [`plumbers_block_ur5e_skills.py`](../../toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py)：公共原子技能中增加连续物理路径、配对力控制、跟踪保护、接触建立和带载交接；抓取与保持使用固定参考，限制累计微调行程。
- [`beam_seating_control.py`](../../toolkits/factory_dual_franka_assembly/beam_seating_control.py)：接近桌面减速、首次接触暂停、根据承重加载或卸载；不直接写零件位姿。
- [`beam_contact_relief.py`](../../toolkits/factory_dual_franka_assembly/beam_contact_relief.py)：确认连续桌面接触后，选择有界的夹紧力减小策略；失去桌面支撑时恢复搬运夹持。它不是落桌成功判据。

出现“目标在动而机械臂不动、下放被保护卡住、接触后压力过大、阶段切换丢接触”时，优先检查目标 TCP、命令关节、实测关节、保护原因和受力日志。

### 场景、对象碰撞与元数据

- [`usd_object.py`](../../internutopia_extension/objects/usd_object.py)、[`static_cube.py`](../../internutopia_extension/objects/static_cube.py) 和 [`对象配置`](../../internutopia_extension/configs/objects/__init__.py)：早期修复中保留的 SDF 碰撞识别/显式配置、实际碰撞体材料绑定、摩擦合成模式和静态工装固定。**这些是公共对象类变更，并不全部限于 Beam**；尤其材料绑定和 StaticCube 构造需要其他任务回归。
- [`公共 canonical 配置`](../../roboassemblybench/tasks/_shared/_fabrica_canonical_ur5e.yaml)：注册扶持技能并使用适合桌面场景的回退背景；[`Plumbers Block 配置`](../../roboassemblybench/tasks/fabrica_plumbers_block/recipe.yaml) 仅同步这个背景回退路径。
- [`factory_cell_tabletop.usda`](../../roboassemblybench/scenes/usd/factory_cell_tabletop.usda)：配套桌面背景，避免旧背景的高架结构落在机器人工作区。碰撞仍由任务物体提供。
- [`canonical_tasks.json`](../../roboassemblybench/assets/Fabrica/canonical_7_bundles/canonical_tasks.json)：当前实验使用的元数据，补充 Panda/Robotiq 兼容性和原始变换字段；Beam 与 Car 的底座候选列表长度也有变化。该文件覆盖七个任务，不能称为只改了 Beam 数据。构建脚本内容与 main 相同，本次没有加入纯权限差异。
- [`已有 canonical 阶段测试`](../../tests/toolkits/test_fabrica_canonical_staged_tasks.py)：收录工作目录中早期调整过的检查。整理时发现 Beam 底座候选 `536` 的净空评分约 `0.18521`、Car 候选 `3657` 约 `0.18702`，低于旧测试要求的 `0.20`。这是候选数据与测试约定之间的待审查问题；不能仅放宽测试来消除失败。

### 机器人执行、材料与空闲臂

- [`UR5e robot`](../../internutopia_extension/robots/ur5e.py)、[`UR5e config`](../../internutopia_extension/configs/robots/ur5e.py) 和 [`scene_builder.py`](../../toolkits/factory_dual_franka_assembly/scene_builder.py)：传递并验证显式参数，找到实际指垫碰撞体，绑定 physics 材料并读回。保留外观材料和原生几何。
- [`gripper_controller.py`](../../internutopia_extension/controllers/gripper_controller.py)：支持物理夹持的命令保持与诊断。
- [`demo_policy.py`](../../toolkits/factory_dual_franka_assembly/demo_policy.py) 和 [`beam_idle_hold.py`](../../toolkits/factory_dual_franka_assembly/beam_idle_hold.py)：对确实空闲且未带载的 Beam 机械臂保持固定的原生命令目标，避免反复把实测下沉位置当作新目标。已带载或正在执行技能的手臂不适用此空闲策略。
- [`beam_arm_drive.py`](../../toolkits/factory_dual_franka_assembly/beam_arm_drive.py)：显式增益试验与实际增益、力矩上限、驱动类型读回。较低增益试验没有保留，当前仍为原先验证过的增益。

出现“摩擦参数不生效、空闲臂缓慢漂移、关节折返、驱动配置与运行不一致”时，查这一组。

### 诊断和物理求解对照

- [`runner.py`](../../internutopia/core/runner.py)：在动作前、动作后、物理步后和观测后调用轻量诊断采样。
- [`pose_mixin.py`](../../internutopia/core/util/pose_mixin.py) 与 [`articulation.py`](../../internutopia/core/robot/isaacsim/articulation.py)：记录显式状态写入，便于确定是否有脚本干预。
- [`beam_physics_diagnostics.py`](../../toolkits/factory_dual_franka_assembly/beam_physics_diagnostics.py)：复制采样、记录物理时钟、实际全局设置、关节状态及写入事件。通过环境变量显式启用。
- [`beam_solver_control.py`](../../toolkits/factory_dual_franka_assembly/beam_solver_control.py)：独立、显式的求解器对照入口。当前默认没有保留 PGS 覆盖。

已有诊断将异常定位到 `world.step` 边界内；采样窗口未发现解释该异常的位姿/速度写入。**这不是“仿真器读取 bug 已确认”的结论。** 数值求解、约束修正、驱动与接触共同作用仍需进一步区分；部分空闲指垫未被诊断缓存覆盖，相关记录不能当作完整有效的双臂传感快照。

## 公共代码影响和审查边界

公共文件确实修改了，不能简单称为“只改了 Beam 文件”。主要限制方式是显式阶段配置、连续物理开关、Beam 对象和空闲状态检查，以及未配置任务沿用原路径。UR5e 材料和驱动支持使用显式参数；诊断默认关闭。

上述限制主要作用于连续抓持和控制分支。对象构造、材料绑定、静态碰撞体、公共背景及元数据仍有共享影响，不能保证其他任务行为完全不变，**必须补充其他任务的物理回归**。当前检查包含未启用策略、其他对象和正常调用路径的测试；没有运行 Car 等任务的完整仿真。建议原作者合并前先审查公共分支，再用自己维护的代表性任务做阶段回归。

为方便审查，本次只移入修复相关源码、测试、说明与精选证据，没有纳入实验目录中的资源删除、无关脚本权限变化或全部历史输出。

## 当前达成与仍未达成

- 已有单种子物理证据：底座真实抓取、脱槽、搬运、与桌面接触；更严格的接触/滑移保护能够拦住不合格动作。
- 空闲臂固定目标已运行验证。v164 末 2 秒桌面平均承重约 `2.391 N`，约为底座重量的 `97.5%`；但严格落桌连续通过步数仍为 `0`。
- 双侧扶持、外绕换扶和最终释放已实现阶段配置；尚未完成完整受力运行验证。
- 仍需解决速度与实际位姿变化之间的差异、持续落桌稳定性和下放请求到关节命令的保护卡住。
- 夹爪外壳/连接显示问题和实机接触参数标定仍待处理。

### 保留和回退

保留：连续物理抓持、指垫实际材料绑定、落桌加载/卸载、空闲臂固定目标、四边界诊断、显式求解器/增益试验基础设施。

回退：PGS 对照、`kp20000/kd1000` 和 `kp40000/kd2000`。PGS 在搬运中失夹；较低增益分别停在抓取位姿检查或落桌保护。不能把这些失败视频当作当前默认控制效果。

最终恢复后的代码通过了历史记录中的 201 项检查和配置读回；**恢复后的完整组合没有再跑一轮仿真**。v164 是保留空闲策略与原增益的代表运行；v166 是最近失败试验，不是最终保留参数的运行。

## 本地检查与复现

不安装 Isaac Sim 也可运行九个 `test_beam_*.py` 的离线逻辑检查：

```bash
python -m unittest discover -s tests/toolkits -p 'test_beam_*.py'
```

这些测试验证接触/滑移门槛、状态保持、控制分支、材料绑定验证、诊断无副作用、空闲目标和试验参数验证；它们不是五零件装配成功证明。

本次在独立提交目录和服务器当前源码上分别重新运行上述检查，**两边均为 201 项通过**。随后在服务器完整 Isaac Sim 5.1.0 配置下运行两个已有 canonical 测试文件，结果 **10 项通过、4 项失败**，耗时约 82 秒。使用现有 Python 环境及 `/tmp` 中的临时 pytest，设置 `ISAAC_SIM_ROOT`，用 `--confcutdir=tests/toolkits` 避开与这两组检查无关的顶层 MongoDB fixture；没有启动仿真、改源码或屏蔽失败。完整日志见 [本机 Beam 检查](offline_tests.log)、[服务器 Beam 检查](server_beam_tests.log)和[服务器 canonical 检查](canonical_tests.log)。

四项失败需要分别处理：

- 候选净空评分：Beam `536` 低于 `0.20`；另行扫描也发现 Car `3657` 低于此值。应审核是否剔除、重新计算，或标记为不参与自动选择，不能只降低阈值。
- 随机化目标集合：新增 `part_6_preseat` 已加入装配区目标组，但旧测试的预期集合未包含它。需要同时检查预落桌目标是否随装配区正确变换，再更新测试。
- 抓取预张开度：实际值为 `1.0`，旧测试按 `robotiq_open_ratio + 0.20` 得到约 `0.54154`。应核对连续物理取件策略，区分 Beam 的显式张开设置与其他任务默认值。
- 可达距离记录：根据当前目标重算为约 `0.574181 m`，选择记录为约 `0.573722 m`，相差约 `0.459 mm`。需检查抓姿调整后可达性诊断是否重新计算；当前仍在 `0.82 m` 范围内，但不能据此忽略不一致。

这些失败不都是物理运行失败，也不能全部归为旧测试过时。当前 PR 为草稿，需审核数据、诊断及对应测试，不宣称全部回归通过。

完整仿真仍需要原仓库 README 指定的 Isaac Sim 5.1.0、Linux/NVIDIA 环境和外部资产。资源下载后，在仓库根目录使用原入口：

```bash
HEADLESS=1 NUM_DEMOS=1 MAX_TRIALS=1 \
  bash roboassemblybench/scripts/generate_fabrica_canonical_ur5e_demo.sh beam
```

严格复现某个历史节点时，需要该节点的配置和源码版本，不能只拿当前代码运行后把结果称为“复现 v151/v166”。精选输出保留配置、部分源码哈希及结果，原完整历史源码快照尚未全部收录。

本次整理后的检查结果见[提交验证记录](submission_validation.json)。实验文本中的个人/机器绝对路径已经替换为符号路径，数值、停止原因和视频没有改变；日志节选明确标注为 excerpt，不能当成完整轨迹。
