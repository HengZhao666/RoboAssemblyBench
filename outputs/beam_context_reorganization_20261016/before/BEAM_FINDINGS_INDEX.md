# Beam 结论与证据索引

## V22 — 壳体修复有原生支持；新窗口错误覆盖短过渡阶段（第22轮完成）

- v199873项候选已完成17130步，exit0但failed=true；503 Beam/252相关、2skip，5本机专项通过。原碰撞mesh及实例保留，新增显示mesh不带CollisionAPI/RigidBodyAPI；原18collider/实例/物理属性与副本点/面/世界变换离线同。
- 初步视觉证据：直接检查已录制4/5/6/7/8秒三视角15帧，原先8秒缺失的两侧黑壳体现在可见；clone下CollisionAPI=false，原隐藏绘制mesh仍CollisionAPI=true且rigid enabled。这只支持该短窗的显示改善，全流程仍待检查。已对齐1252个前2500步物理边界的q/native目标/可用body位置最大差0；873源码checkpoint差异0；完整三视频2141帧各已下载，SHA同、全解码0error，30/40/44秒及末段所查壳体也正常。
- 已验证首件范围：10950步支撑约2.300N、位姿联合96放行，原original_seat_ready=false；9656条截至10950的同key物体/TCP/夹爪q/双侧力记录与v194全同。
- 判定范围错误：post_lift_restore_gravity原360步=1.5秒、min48，2秒窗需480步；终止17130时原条件true、短窗true、长窗仅313样本/1.3秒未满，样本/COM有效，无latched failure。首柱双侧约20.62/19.77N、滑移2.203mm，34组phase/object保持统计invalid0；本轮失败归因于新判定范围，不作物理失夹证据。
- 已另证：首柱peel保持在15922原条件及位姿窗均true；到15441前19984同key接触数值全同v194。2141实体样本均44poses，跨臂/持件对他臂事件0仅该范围；scene背景USD根不在声明48枚举内，不能叫全场景安全。
- 合并诊断：同一轮继续原配方位姿差分探索，首柱后无行政停止；中间静态与最终静态/支撑使用既有双窗口，各体同刻采样，保留原raw结果。后段全部物理关卡尚未原生通过，不称完整装配成功。
- 新离线路径：左张开81、退避281、park352、transit221，以及柱2实际限幅预对齐932、两模型move_above各243样本，新增非目标重叠0；完整48collider/44pose/5件参与，持柱3右臂固定在名义侧抓终态。最小跨爪9.910mm是名义值，仍缺真实释放/受载/servo/cooked/CCD/defaultoffset验证，不能认证未执行后段安全。
- 入口：round22_full_continuation/source_manifest.json、shell_offline_audit.json、left_release_screen.json、left_part2_screen.json、live_visual_prefix及其相机clock；原始数据保留服务器v199输出及本地完整后审包。

## V21 — 壳体可见元数据不能证明画面可见（2026-10-14）

- 范围：本批第1/10轮、累计21，v197记录器初始化失败原件保留；修正实际机器人映射后，同源码v198跑2500步至底梁lift，正常退出，非装配成功。source872运行前后同；三完整原视频下载并两端逐SHA验证，入口round21_shell_continuation。
- 候选/否定：壳体visuals解除实例并绑定不透明显示不足以修复。直接看三视角0/4/5/6/7/8/10秒，正面8秒仍缺壳体；314壳体采样的visibility inherited/opacity1、rigid enabled且mesh相对base变换稳定，不能据这些元数据宣告视觉通过。视频clock936条/三相机各312，ReferenceTime有效，固定偏移约−1.020833秒是不同起点记录，不自动命名延迟/新鲜度认证。
- 物理回归：与v194前2500步1196条同key接触记录的物体/TCP/夹爪q/双侧力全同；1252四边界实测q、原生目标及可用直接body位置最大差0。312实体样本均44poses，原报告无error；仍有四VisualCube腿无collider、45默认offset、cooked/CCD/自碰撞未证，以及Lula9夹爪frame和未来夹爪冻结缺测。仅前缀，不泛化到长turn或后段。
- 重要修正：原壳体base_link/visuals内mesh本身带CollisionAPI，解除实例改变碰撞实例表示，虽离线形状/物理属性相同且本前缀原生同，也不能写成全场景纯视觉表示不变。下一候选保留原实例，只加不带CollisionAPI/RigidBodyAPI的显示mesh；无碰撞副本的点/面/世界变换及18原collider/实例/物理属性离线逐项同，尚待新原生画面。
- 证据：round21_shell_continuation/evidence/completed_analysis.json、contact_prefix_comparison.json、原runtime/stage报告、shell_visual_trace和video_frame_clock；review_bundle SHA22c6a7cff78d3fc7989bbc0673305a5108d32042fe9e5921bf38f607e6de14d8。native_prefix_clock_review.json服务器保留。本轮严格落桌/首柱后换手/释放/完整五件未验证。

更新：2026-10-14，Asia/Shanghai。累计24轮后审完成，本批4/10已完成；最新仅一轮v201授权已执行，现暂停，不启动25。正式五件装配仍未成功。历史准备/运行记录保留其原始上下文，最新授权、版本和范围以当前交接及各完成条目为准。
此文件是长期知识索引，不是第二份交接。当前状态只读 BEAM_REPAIR_HANDOFF.md；新对话按瓶颈读取本索引相关ID及原始证据。

## 记录规范

每条记录注明命题、对象/阶段/版本、改变因子、证据等级、支持或否定结果、适用范围及尚未证明事项。保留否定结论和后续反例；原始日志不删除。原生局部对照、原生相关证据、离线反例、静态推论、待验证假设分别标记。测试数量不替代任务能力。

## 原始版本与归因边界

原仓库历史main基线为4b5a89410c5cd571d218d58da5f65a16682a1a23；它含fixed_joint、锁件、碰撞控制及释放冻结，成功标准与当前纯接触物理模式不同。不能把“原版演示成功/失败”直接当真实抓取消融。初期改动在Git中聚合，逐轮source_before/after、changes.patch、配方及原生轨迹才是归因资料。

本次只读核验v138/139/140/141/143/148/149/150/151九轮，共162个历史源码快照成员与各轮manifest匹配。这是相关九文件快照的字节验证，不是完整环境或全源码冻结。

## G01 — 接触存在还需弱侧承载余量

- 对象/范围：底梁6闭合；v135/v136/v137，seed0/layout505，真实物理模式。
- 改变因素/反例：v137恢复单侧确认时序，使弱侧余量分支可触发；v136未触发该分支，不能当作该机制无效的反例。
- 证据：原生局部对照。v135最长共同43帧；弱侧降至0.891N中断1N门，接触/位姿仍有效。v136新增余量分支0次，确认最长3<4，不能据其失败否定分支效果。v137恢复原单侧确认时序，首次余量动作task1686前五类轨迹与v135全同；之后7次余量动作，最长97帧，超过原96门。
- 范围：证明局部闭合资格改善；登记后仍失夹，没有抬升或完整抓取。
- 可迁移：先验证分支触发、实际发出、物理效果，控制余量2N和成功门1N分开。不能机械复制25µm参数。
- 原始入口：[历史诊断](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/beam_contact_balance_review_2026-09-30.md:25>)、[前缀对照](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/fabrica_beam_support_v137_isolated_margin/runtime_baseline_comparison.json>)、[控制对照](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/fabrica_beam_support_v137_isolated_margin/control_comparison_audit.md>)。

## G02 — 纯物理抓持登记必须只读

- 对象/范围：底梁首次登记；v137→v138。
- 改变因素/反例：删除登记对物体位姿及速度的写入；v138后续hold仍失夹，说明只读登记并不足以保证阶段连续抓持。
- 证据：原生局部对照。登记前至task1885位置、arm命令、物体位姿和力一致。旧登记仍写物体姿态/清速度；去掉这些副作用后连续稳定从97到168，包含96资格及登记后72保持。
- 范围：成功通过闭合；进入下一hold仍失夹。不能凭pure模式名称证明没有状态写入。
- 原始入口：[记录](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/beam_contact_balance_review_2026-09-30.md:49>)、[登记前对照](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/fabrica_beam_support_v138_readonly_grasp/registration_baseline_comparison.json>)。

## G03 — 阶段连续性要核最终关节和夹爪目标

- 对象/范围：底梁close→settle→peel/lift；v138–v143。
- 改变因素/反例：先继承最终arm/gripper命令，再从上一命令严格求解并禁止实测q回退；v140笛卡尔请求连续但仍失夹，反驳仅核请求位姿即可证明连续性。
- 证据：原生多阶段对照。v138重建零偏移hold产生0.591mm TCP、0.00507rad腕命令跳变后失夹；v139继承最后实际命令，72帧hold通过。v140笛卡尔请求连续，但IK回退实测q，仍失夹。v141从上一命令q严格求解、禁止该回退，首次真正提拉30.536mm，无失夹；后续局部有界IK减少分支跳变。
- 范围：多个逐步修复，不能归因单个参数；v141仍抬升超时。计划TCP、构造q、最终返回动作、SDK发布、原生target、实测q/FK必须分别记录。
- 原始入口：[逐轮记录](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/beam_five_rounds_2026-09-30/review.md:5>)、[IK与抬升](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/fabrica_beam_support_v141_strict_ik/ik_and_lift_audit.json>)。

## G04 — 执行中不能折回真实关节状态

- 对象/范围：底梁转向；v146→v147。
- 改变因素/反例：移除真实关节折回和速度清零；v147仍未完成落桌，改善只适用于越过旧转向断点。
- 证据：原生局部对照。故障前6669条动作一致；移除运行时关节折回/速度清零后越过旧故障，完成转向及悬停。
- 范围：目标角度选择和写真实关节状态不是同一操作；只证明该边界，不证明落桌。
- 原始入口：[记录](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/fabrica_beam_support_v147_continuous_joint_state/review.md:9>)、[连续性对照](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/fabrica_beam_support_v147_continuous_joint_state/continuous_state_audit.json>)。

## G05 — 带载阶段持续受限力调节是完整搬运关键候选

- 对象/范围：相同浅抓方案的底梁保持/抬升/运输；v149→v150，并有v151复核。
- 证据：原生强相关，尚非唯一因果消融。v149约3mm起吊即滑移3.023mm，两指仍有约3.00/3.84N；v150把已有4–12N受限调节延续到持载阶段，完整起吊、转向、下放及伴随保持通过。下放阶段最大相对滑移0.975mm、双侧最小力8.36N；该力下界不代表整个起吊/搬运窗口，起吊阶段记录的最小双侧力约0.598N。原滑移和真实接触门未放宽。v151前2700步四类日志与v150一致。
- 归因限制：v150还修探针路径/摩擦查询；v149/v150在加载控制前已有分歧（task905位置约1.05µm；task1392力6.79/11.38与4.03/5.78N；task1465 TCP约65µm）。四元数q/−q等价，不能按符号判断大旋转。尚无同一成功基线关闭/恢复该项的成对探抬消融。
- 范围：底梁仍悬在桌面约2.48mm，未证明稳定落座、卸载或完整装配。40N是控制/资格阈值，不能据配置宣称原生瞬时力永不超过40N。
- 原始入口：[v149](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/fabrica_beam_support_v149_local_close_ik/review.md:3>)、[v150](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/fabrica_beam_support_v150_loaded_force/review.md:3>)、[v150审计](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/fabrica_beam_support_v150_loaded_force/precision_audit.json>)、[v151前缀](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/fabrica_beam_support_v151_pick_frame/prefix_2700_audit.json>)。

## G06 — 重定位和保持不能暗中中断接触

- 对象/范围：底梁距目标50mm的重定位与伴随保持；v147/v149/v150。
- 改变因素/反例：停止该距离边界的夹爪碰撞过滤；探针有效且读零不能反证全环境无碰撞，有限采样不能证明完整扫掠安全。
- 证据：真实记录重放及后续原生覆盖。旧重定位在50mm内请求夹爪碰撞过滤；v149修复，v150在task8945实际越过边界且接触保持。
- 范围：外部接触探针句柄/有限值有效不等于全环境覆盖；不能由零读数排除碰撞。
- 原始入口：[v149说明](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/fabrica_beam_support_v149_local_close_ik/review.md:9>)、[v150覆盖及边界](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/fabrica_beam_support_v150_loaded_force/review.md:9>)。

## G07 — 好的初接触几何可减少居中需求；参数不能跨件照搬

- 对象/范围：v149/v150底梁浅抓与当前柱3。
- 证据：原生可行例+当前配置审查。v149/v150实际居中动作0次；底梁采用局部Z+28mm、pad reach120mm、特定轴对齐/偏移。底梁局部Y厚17.78mm，柱3约12.003mm，柱3默认reach112mm、抓姿和工装关系不同。
- 范围：不能证明28mm是唯一成功因素，也不能直接复制给立柱。需检查实际pad与侧面覆盖、全开到闭合扫掠及非目标接触，再选候选抓位。
- 当前实现：共同抓取生成器已经共享指垫补偿、全开接近、严格双指资格、命令连续、持载力调节。柱3在v184已双侧闭合；G05持载路径在v185延续至lift/脱槽，后续转向仍失败，见G09/C07。
- 原始入口：[历史源码审查](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/beam_method_review_20261012/first_grasp_history_audit.md>)、[当前连续模式策略](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/submission/beam_upstream_integration_2026-10-10/roboassemblybench/core/beam_physical_compiler.py:4669>)。

## C01 — 原落桌问题与探索放行分别报告

- 证据：v169/合并版同条件到达搬运但原落桌超时；v180/v181/v182位姿探索门推进，原raw共同门仍0。v182落桌锚点10855、共同96在10950；全部已观测postseat guard成立，但仍在柱3闭合失败。
- 对象/阶段/因素/反例：底梁落桌，v169→合并版及v180–182；探索位姿放行改变了进入下一阶段的条件，原速度共同门0是严格落座仍未通过的反例，不能用于宣称速度getter有误。
- 范围：探索通过不证明原速度门、最终卸载稳定或五件装配。中性box力/速度校准尚未执行，不能宣布原生getter错误。
- 原始入口：[当前完整重放](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/beam_runtime_repairs_20261012/round09/v182_analysis.json>)、[精简重放](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/beam_runtime_repairs_20261012/round09/v182_exploration_compact.json>)；v169/e6资料入口见归档交接。

## C02 — 当前闭合：尾段回中停止与新运动确认超时

- 证据：v180旧开度下饱和，放宽后v181实际越过旧q=.736但没有触及新上限/seek cap，没有证据支持以进一步加大行程解决本轮问题。v181整轮发出4次回中、3次双侧取消，末次回中task14773；末645帧（task14780–15424）single/margin确认最长3/1，模式切换重置，不能把尾段统计解释为整轮从未回中。v182连续4后计划一次25µm，普通pending实际发布与SDK/native一致。
- 对象/阶段/改变因素：柱3闭合，v180→v181仅放宽夹爪最低开度；v181→v182为弱侧连续确认。未触新行程上限和仍无双侧力分别是继续增行程/仅修确认就能夹稳的否定证据。
- 真实未夹紧：665次左探针有效但力0，native separation正0.30–0.72mm。资格0/96；原生运动确认最长6<8，240ticks/1秒后失败。
- 归因限制：pending完成点与已发q的模型FK相差约200µm是事实，固定bias正对照能完成，所以不同目标本身不是根因证明。239普通pending的模型FK距笛卡尔请求0.21–1.16µm；未发现命令丢失或本轮IK精度不足。力调节与PD偏差变化仍是待验证假设。
- 原始入口：[v181对账](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/beam_runtime_repairs_20261012/round08/v181_post/v181_post_analysis.json>)、[v182最终审查](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/beam_runtime_repairs_20261012/round09/v182_pending_final/v182_pending_motion_analysis.json>)。

## L01 — 尚未发出的动作不能记成物理进度（第01轮已修）

- 证据：离线完整act链反例，使用SDK线性替身，不是原生试验。planning发生在旧arm动作构造之后；下一tick80N取消时，旧运动投影可被算成新动作100%已执行，并首次返回未发出的新位移。
- 适用件/阶段/版本：柱3weak-side闭合回中，b8反例→dcb4b73修复；其他件legacy回中和全部持载/插入阶段未使用新生命周期。
- 唯一改变因素：区分planned/final-action-issued；首次最终返回新arm_q时记录该tick测量起点与最终q的模型FK，增量完成点从实际issued起点计算，SDK/native发布另行对账。未issued取消进度为0；计时不计planning tick。固定anchor、收费预算、40N减压和双侧真实资格不变。
- 验证：完整act线性反馈反例中首次发布前80N，从旧误记100%并返回新25µm变为0%且不返回新位移；瞬态300µm跟踪误差不再把planning旧起点当issued起点，能完成增量。对齐与固定300µm bias正对照仍能完成。43专项、服务器437 Beam/237相关pytest通过，2 skipped；最终限幅q的FK被记录、FK不可用时拒绝返回新动作。
- 反例与未验证：两端IK偏差、随夹爪调节变化的PD偏差仍可影响实际小位移；本修复不证明v182失败唯一根因或双侧抓稳。未有原生硬过载取消覆盖、多seed或其他件验证；v183普通发布对账见下。旧原始反例保留。
- 新证据：[第01轮离线前后对照](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/beam_grasp_repairs_20261012/round01_safety/after_counterexamples.json>)；服务器同轮unittest/pytest与source_manifest、deployment_report保留。
- 第02轮原生范围：v183 observed planned14648、最终returned14649；84普通close样本SDK与final hold_q差0、native目标最大差1.183e-7rad。源码前后861零差异。未出现40N过载，不能宣称原生取消分支已覆盖；柱3仍左力0/双侧0、最长运动确认7<8，否定“仅修生命周期即可夹稳”的泛化。第一件成功回归另存完整比较。
- 原始入口：[完整链反例](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/beam_runtime_repairs_20261012/round09/pre_round10_review/independent_pending_lifecycle_probe.json>)。

## L02 — 不用确认窗口最后低相位代表扰动基线

- 证据：离线实际helper反例。确认窗口强侧约7、7.3、7.3、0.659N；最后瞬时imbalance0.359N会把原有7N高相位判作恶化。窗口peak7能容纳原波动，真正超限增大仍被原1.5x+2规则拒绝。
- 对象/阶段/改变因素/反例：柱3weak-side pending扰动判定，b8 helper；仅替换确认窗口瞬时基线为窗口peak的离线对照。真实15N增大仍触发判定，是不能任意扩大扰动容忍范围的反例；正常夹爪目标保持尚无原生对照。
- v184结束后第04轮独立实施窗口peak，只作用weak-side；正常夹爪目标保持仍未实施，两者分开。46专项、服务器440 Beam/237相关pytest通过、2 skipped；时钟gap/强侧反转/无效probe/40N重置/新窗口/重复tick6项明确检查通过。代码修复已提交31bb67b，原生完整v185待验证。
- 新离线范围：按dcb生命周期显式构造final-action-issued和连续8个motion-ready样本，旧0.359N基线把原有7N波动判作恶化；候选peak7N不误报，真实15N仍触发原1.5x+2规则。首次复用旧helper探针停在planned而未检验扰动分支，该输出保留但不作修复证据；有效反例为l02_offline_proposal/counterexample_with_issued_contract.json。新窗口回到5.7N而不携带上一窗口27.7N，重复tick不积累，尚无原生新版本结果。
- 减压、缺测及过载保护保留；不因此放宽40N门。一般IK两端各5µm也可形成10µm增量误差，但本轮native误差小，不能据此归因。
- 原始入口：同L01，字段confirmation_window_imbalance及within_global_ik_tolerance。

## L03 — 向下合力不能取绝对值当承重（第05轮已部署，原生反例未覆盖）

- 对象/阶段/版本：31bb原combine_support_surfaces→table_support_ready，底梁初落桌、支持watchdog及最终load读数；离线真实helper链、合成接触向量，没有新原生结果。
- 原反例：gap=0、raw线角速=0、有效pair，向下Fz=-2.4525N汇总为+2.4525N并被原minimum=1.962门接纳；[+0.5,-3.0]净向下-2.5也接纳。现有探索另读signed方向保护，不能推断普通原门同样保护。
- 唯一因素/正对照：候选保留signed sum并对向下合力返回valid=False，经原采样dict合并传给控制器；两向下例被拒绝，实际96-sample helper原本累计96的向下例改为0；20项本机CoordinatedSupport候选回归通过（临时测试ROOT定位第一次3错已修，原失败记录保留）；原落桌控制器对候选负力样本立即failed/delta_z=0。+2.4525和[+3,-.5]净向上例仍通过，低于门的1N仍拒绝。40N、速度、间隙及96/240条件未动，没有补入未校准friction或缩放力。
- 状态/范围：原草稿l03_offline_proposal保留；第05轮已独立部署edd661d并服务器回归通过，见V05；v185运行原始来源仍保持31bb。未找到本轮真实向下合力事件，不把此缺陷当raw速度或旧载荷缺口根因。完整vertical_force消费者链已审并独立回归部署；没有原生负力反例覆盖。

## C03 — 同轮两件覆盖与非目标接触比较（新增原始观测）

- 对象/阶段/版本：v182 b8，底梁6与柱3的全开接近、闭合及第一件保持/搬运窗口；没有改变物理因素，是同轮已测对照而非因果消融。
- 事实：底梁Y厚17.7800494mm，572个close样本，真实双侧≥0.1N及双侧≥1N+严格接触最长各168步（包含登记后保持）；控制最长稳定168。柱3Y厚12.0033237mm，665个close样本，左力全部0，右峰15.9646N；双侧最长0、控制稳定0。成功和失败不能只用夹爪q相同或TCP偏移相同比较。
- 覆盖代理：调用当前源码的15点面采样常数，按原生finger/object姿态重建。底梁最后close每侧2/15点在目标Y侧面无margin矩形内；柱3最后每侧1/15。该计数不是实际指垫面积，盒代理Y面与原生接触点不一致，不能由采样点“在盒内”证明已接触。
- 原生反例：柱3首个有载接触task14641在局部z约27.47mm、y约+4.68mm，目标bbox半Y约6.00mm，提示必须核真实网格表面而非把12mm盒厚当处处相同。柱顶附近零力点也有正separation；接触点存在不等于法向力存在。真实网格与完整扫掠仍在审查。
- 非目标证据：已有柱3窗口记录的支撑/邻件探针无>0.01N，但每8步整爪采样、未覆盖的对象对/无效探针和步间接触不能据此排除。v183新增独立只读9链接×全部已登记场景对象采样；未验证范围包括原生烹制碰撞网格、面承载面积、完整连续扫掠和其他seed。
- 回归原始证据：`L/outputs/beam_grasp_repairs_20261012/reference_v182/reference_manifest.json`，17文件服务器/本机SHA零差异；原733MB contact日志SHA等保留。分析入口：[同轮比较](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/beam_grasp_repairs_20261012/geometry_review/reference_comparison.json>)、first/second_close_geometry.jsonl；三原视频保留在round09/v182。没有据此宣布柱3抓位深度假设成立。

## C04 — 柱3当前接触处是较窄截面，不能把bbox厚度当局部厚度

- 对象/阶段/版本：v182 b8柱3初闭合的有载点与两件原USD；改变因素为只读三角网格切片与原生姿态重建，没有修改物理。证据等级：已核实际资产的离线几何事实，不是原生深度对照。
- 证据：柱3接触盒局部z=27.47mm截面Y=9.4318mm/X=5.5877mm；z=23.47mm截面Y=12.0033mm/X=11.5029mm。底梁对应切片Y=17.7800mm，当前局部差异大于仅比较全局17.78/12所显示的差异。真实面法向/承载面积仍须用接触证据核验。
- 指垫事实：每侧真实碰撞mesh有finger4step与fingertipsstep两块，均convexHull；前面在body局部Y约44.908/43.548mm两个平面，前者Z=103.920–110.420mm、后者110.420–148.420mm。当前15点统一43.55mm平面和95–145mm范围是代理，不能替代两块实际面。原生烹制后网格尚未导出。
- 可证伪候选：仅柱3grasp_object_frame_offset局部Z=-4mm，将当前接触深度移向较宽截面；第03轮v184局部结果见G08；几何事实与原生支持分开记录。编译默认0；非零时只变move_above/descend/close三姿态，其余158阶段/objects/targets不变，真实目标变换为对象局部Z4mm且不变方向；437 Beam/237相关pytest通过、2 skipped。布局IK替身不证明路径可行。其“支持”需实际下移且双侧接触/夹稳改善、无新非目标碰撞，改变成功门/延长超时不算支持。
- 反例与未验证：底梁浅抓成功反驳普适“浅抓必失败”；较宽面不必然产生双侧法向力。PD/夹爪力耦合、偏心接近、实际扫掠、SDF/convexHull求解、提拉与完整装配未验证。连续两次同路线无能力改善则停线重审。
- 原始/派生证据：[实际网格比较](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/beam_grasp_repairs_20261012/geometry_review/authored_mesh_comparison.json>)；pad_mesh_with_targets.json包含冻结顶点/三角形、原USD路径/哈希与导出SHA。切片坐标扣除原bbox中心，与contact_box_offset代码和原始中心核对。候选判据见round03_depth_candidate/experiment_contract.md。

## C05 — v183安全修复未改善原抓位的双侧能力

- 对象/阶段/版本/因素：柱3闭合，b8/v182→dcb/v183；唯一控制改变为L01生命周期安全修复，几何/力调节/成功门/240步运动超时不变，完整物理前缀重放，附加只读整爪采样。
- 原生结果：666闭合样本左探针全有效且力0，左最小native separation正0.3005mm，右峰15.9646N；双侧最长0、共同稳定0/96。末task14889故障、总14890步。最长运动确认7<8，相比原6属于诊断变化，未改善夹稳或进入抬柱阶段。
- 支持/反例：支持L01首次最终命令和SDK/native对账；反驳将该安全修复当作当前单侧接触根因修复。既有对齐/固定bias正对照仍保留，不能只凭完成点和模型FK差约200µm宣布根因。
- 非目标接触：9链接×已登记对象的每8步只读探针未见非目标>0.01N；每对有效性/容量单列，有限间隔不能证明连续扫掠无碰撞。
- 未验证：硬过载原生取消、局部Z候选、正常夹爪目标保持、L02窗口基线、插入/释放/完整装配/多seed仍未验证。
- 证据：[v183完整原始分析](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/beam_grasp_repairs_20261012/round02_native_baseline/v183_analysis.json>)；服务器原始输出路径C/outputs/fabrica_beam_grasp_v183_safety_baseline、SDK/native逐行对账和原日志SHA均保留。源版本dcb与b8回退仍有效；没有为无能力进步的v183另建源码检查点。
- 第一件回归最终范围：v183与v182选定5157条第一件观测匹配，位置/TCP/q/双指力全部差0，覆盖close/settle/peel/prime/hold/lift/place/探索setdown选定窗口，不冒充全部源日志/所有感知量一致。完整比较见round02_native_baseline/prefix_comparison_final.json。

## G08 — 柱3局部下移4mm使既有回中推进至真实双侧抓持

- 对象/阶段/版本/条件：柱3初抓、保持、短peel；v183/dcb原抓位→v184/b5a0968，seed0/layout505、Isaac5.1/TGS、原材料/驱动与力调节。唯一主要因素为local Z=-4mm初始抓位；只变move_above/descend/close，其余158阶段、物体、装配目标及原成功/超时门不变。L02修复在此结果之后才实施。
- 实际执行：全开指垫左右分别下移4.0076/3.9931mm，满足预写≥3mm判据。首有载点下移2.8570mm，因闭合关节位置随厚截面变化，不能把TCP4mm机械当作每个接触点4mm；首有载点局部z24.616mm。实际指垫和有载点分别核验，不改写预写判据。
- 原生结果：794个唯一close task步（1025原始记录，登记后重复已去重），双側≥0.1N最长175、双側≥1N+strict最长168，原96门通过；controller稳定168，左峰2.0389N/右峰11.4831N。回中计划16次、原8步运动确认达标；未改240超时。与v183左力0、双侧0形成同条件对照。99 ordinary close样本SDK差0/native目标最大差1.192e-7rad。
- 夹稳/短提拉：额外grip_settle73唯一步全valid；peel191唯一步全valid，物体世界Z实际+8.0457mm、总位移8.1833mm，最大相对滑移1.8205mm<原3mm门；prime与后续hold-check73/97唯一步全valid，最大后续滑移1.9003mm。进入lift入口15446结束，failed=false、local_probe.completed=true，full_assembly_success=false。不是后续lift/搬运/插入证明。第04轮核真实编译phase后纠正先前80mm印象：柱3lift名义60mm、原位置容差25mm，阶段通过仍须报实际位移。
- 回归/非目标：第一件5157选定观测仍差0。9链接×已登记对象每8步未见非目标>0.01N；未覆盖机器人刚体对、步间接触、原生cooked mesh面积和全连续扫掠，不据零采样排除全部碰撞。
- 归因与反例：支持此条件下更深初接触几何可让现有回中建立双侧并局部夹稳，不证明宽度或厚度是唯一子因素，也不证明所有零件浅抓无效（G07底梁成功为反例）。其他seed、其他零件、L02版本、完整后续及原严格落桌/最终卸载均未验证。
- 原始证据：[v184分析](</Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench/outputs/beam_grasp_repairs_20261012/round03_depth_candidate/completed_summary.json>)、hold_lift_validation.json、actual_depth_validation.json、open_pad_depth_validation.json、prefix_comparison_final.json；服务器C/outputs/fabrica_beam_grasp_v184_depth4原日志/三视频保留。运行前后861源码零差异，v184原启动清单d3f87不追改Git标签。
- 实质进步保留：b5a0968eb7f7e5ff7dfe5925d6726adb0f1f8b8a；R/grasp_repairs_20261012_20rounds/retained_b5a，861源码tar SHA74a3b2984d8c778e4b16b3a648f83c2c152bbc8284adae258b7ccf82dec19b02，运行override=-0.004。default0恢复原抓位；新控版本31bb不改写此基线经验。

## G09 — 柱3夹稳能力已延续到抬升与脱槽（第04轮原始后审完成）

- 零件/阶段/版本/因素：v185/31bb，柱3close→settle→peel/prime/hold→lift→pickup_clearance；继承已成功局部Z=-4mm，独立L02窗口peak修复；扩大后续运行和整爪诊断覆盖，不能把首次后续通过唯一归因L02。
- 已测里程碑：close最长原双侧1N资格168，通过96门；close结束task15001，前一v184为15015，16次recenter计划。此14-step时序差不是单独性能因果证明。后续所有已观测保持有效，lift15428–16177物体世界Z+35.2342mm；下一pickup_clearance16225–17089完整段再+84.893227mm；原partial文件31.5826mm保留但不作终值。先前peel实际+8.0143mm。lift原请求offset60mm/容差25mm，未改门，不能将通过阶段写成真实60/80mm上移。
- 第一件回归：5157选定close/hold/peel/lift/place/探索setdown观测全部匹配v182，位置/TCP/gripper q/双侧力差0；其他状态不在这一比较证明内。
- 反馈/安全边界：截至17576，柱3保持valid、滑移约1.974mm<原3mm，双侧约8.77/7.68N；底梁支持guard成立。随后在18770/assembly_clearance报position_slip=3.2080633mm，orientation_slip=.0428491rad，双侧8.38294/7.24904N；失抓generation守卫停止，总18771、进程exit0。未到transport_hover/插入/换手/释放/五件。原始完整后审已完成，见C07；SDK/native四边界覆盖缺口见E01，普通contact expanded另有SDK动作/关节与9链接记录，不应称完全没有执行证据。
- 原始入口：round04_l02_full/column_actual_motion_partial.json、column_partial_review.json、prefix_comparison_first_grasp_complete.json；服务器原contact/hold/control/gate日志未删除。

## C06 / H01 — 有双侧力仍触发保持拒绝；先核对判定帧

- 已知原始失败/版本：v185/31bb，柱3assembly_clearance18770，position_slip=3.2080633mm超过原3mm，orientation_slip=.0428491rad低于原5deg，双侧8.38294/7.24904N。仅position_slip失效；不能将本轮停止解释为缺少双侧接触。观察来自原始hold/guard末记录，完整窗已核验，后审见C07。
- 独立代码事实：连续Beam的_preserve_beam_v169_task_pose使_get_robot_task_pose选measured-joint Lula FK，_current_relative_pose及保持原点位移用此帧；contact expanded另记录实际base_link/SDK动作/关节。这里是既有控制/抓持接口保留，并非本轮新增。
- 假设/预写结果：若可信同帧本体相对姿态稳定且模型↔本体变换漂移解释超门，支持判定帧问题；若两帧都超门而校准变换稳定，否定此解释，再比较真实滚动/滑移、轨迹载荷、指垫几何或碰撞。不能直接全局更换控制坐标系、重置锚点或放宽3mm/5deg。
- 原始历史反证：v182第一件646个expanded观测，登记后校准变换位置变化最大0.694µm、旋转变化最大约6.91e-7rad；直接姿态差约π是固定坐标转向，不能称180度抓持误差。第一件未支持“所有阶段都有大幅模型↔本体漂移”。旧柱3失败前455expanded未登记，不能据其无参考锚点推断本次转向。
- 未验证范围：现有body valid标存在，不独立认证native handle/读数时点；v185末窗口及材料点/转动分量已计算（C07），否定本轮帧漂移解释，原body handle独立认证仍缺少，不称物理指垫确已滑动3.208mm。同帧对账已完成；保留原预写假设作为被否定记录，慢转向候选见C07。见round04_l02_full/postmortem_decision_contract.md与historical_recorded_frame_comparison.json。

## U01 — 进入完整流程前的仓库范围审查与未触发范围

- v184局部通过后，检索primary runtime/compiler/policy/robot/collector的状态写入、固定/柔顺关节、碰撞、终端skip和grasp-offset使用点，实读pure注册/state_sync/release、UR5e连续角守卫、真实attachment优先的插入转换、collect与kinematic replay及正式/探索报告分支。不是第三方vendor逐文件完整安全审计。
- 161编译阶段（原控制快照+已核3深度delta）未见kinematic_carry、非pure attach、启用碰撞过滤、关闭对象碰撞/重力或终端skip配置。pure注册先存只读抓持关系、state_sync跳过传送，UR5e preserve_continuous_arm_joints跳过真实q折回/清速度；legacy/可视化重放setter仍存在，不可运行其路径来证明当前成功。
- 插入真实pure状态跳过无固定关节可转换的柔顺逻辑；该修复有离线反例/测试，尚无此路径原生覆盖。原下游插入/释放门、跨阶段seek预算与中性box速度/力校准仍未决；旧详细交接及原readonly_counterexamples.json保留。
- formal_success在BEAM_EXPLORE_POST_SEAT=1下显式false，完整流程须单列探索与正式资格。原速度门与最终卸载240步不因局部通过自动成立。审查范围记录round03_depth_candidate/stage_switch_repository_review.json。

## E01 — 未设置阶段筛选不等于全阶段诊断

- 范围/版本/阶段：第04轮v185 launcher，31bb；只读配置审查，无物理因素变化。
- 事实：BeamPhysicsDiagnostics构造函数PHASES缺省为base_6_set_down；unset仍取缺省。此前运行宣称SDK/native全阶段对账准备不成立。接触/运动/保持/受限力/整爪日志独立记录，不能用它们替代缺失的SDK/native目标或setter调用证据。
- 修正/未验证：本轮源与原配置冻结；分析显式标边界覆盖仅底梁落桌。下一轮PHASES='*'、END_STEP=-1，再由原始文件验证实际覆盖。仅配置读取事实，不说明物理结果退化；没有因为缺少记录宣称没有状态写入。
- 原始入口：round04_l02_full/diagnostic_coverage_audit.json、run_full_assembly.sh与beam_physics_diagnostics.py:416。

## E02 — 取消局部停止不等于取消已有探索范围

- 对象/阶段/版本：第04轮31bb/v185 launcher+full worker，beam_exploration原守卫；只读代码范围核验，不修改物理。
- 原始事实：STOP_PHASE=assemble_00_part_3_physical_hold_check，FORBIDDEN_RELEASE_PHASE=hold_6_to_3_switch_release；BEAM_EXPLORE_POST_SEAT=1沿用此守卫。full worker没有v184 lift入口局部停，但仅能继续首柱完整抬升/运输/插入/保持，不能进入之后换手释放和其余三件。
- 否定/纠正：否定“移除短抓取停止后当前配置就能跑五件完整流程”。v185原合同和launch清单不改写，另追加scope_correction_20261013.md。首柱探索成功不能当五件或正式成功。
- 未验证/下一步：原底梁raw速度共同门和中性box校准未解决；不能直接绕过守卫制造完整装配。正式模式关闭探索后才可覆盖后续/最终卸载，原严格门保留。

## S01 — 被动场景切换PGS没有恢复原承重/速度资格

- 零件/阶段/版本：v175，ce8daf32bfde2d66d43f20b231523c970d5b0ab4；底梁6与原静态支撑、无机器人及其他动态件，CPU/240Hz/请求16-4/external=false，0.25kg、原SDF/材料、初始底隙4mm。
- 单因素：TGS→PGS；PGS不调用TGS-only setter，原生回读external=false/CPU/重力一致。不同求解岛不代表完整机器人场景。
- 原始证据本次重算：两组各960步，末240平均支撑normal为0.55412399/0.50514713N，平均raw角速0.06848505/0.09082934rad/s，原联合门最长均0。raw SHA分别6071d1b1df724c87ab817362150a54a0b16b51916c651ef5f794abcdf1ea5c03、6f5e0db95ea4f7f989ba022741c931a04b647bd96ad50e233329f92cdd791c49。构造/三边界回读SHA、原首末样本见measurement_calibration_preparation/historical_raw_verification.json。
- 否定/范围：不支持把切换生产solver当直接修复；PGS位姿excursion较小不等于载荷/速度共同门通过。报告力缺口不证明getter错或完整外力守恒违例；sleep状态没有有效现成getter。

## S02 — 固定已发布目标也未恢复底梁严格落桌门

- 对象/阶段/版本：v179/cfbdc475d4927a710b8acefa04231f3229070466，完整机器人场景base6落桌，2秒固定已发布arm/finger目标；与G03命令连续性不同，是落桌候选因果排除。
- 证据：历史完成后的v179_analysis.json（本次保留原文件，不宣称新仿真），task10321–10800共480物理帧；四边界SDK position目标常量差0，velocity/effort目标0，COM excursion约4.64µm；原raw联合最长0，载荷合格267/480、最长78<96。原生角速最小0.0357>原0.03门。完整结果仍落桌超时。
- 否定/反例：排除该固定窗口中持续IK/Z/夹爪目标修改或SDK目标覆盖作为唯一原因；也否定“仅去掉raw速度即可通过该窗口原共同门”。位置目标不变仍可产生drive/接触反应，采样位姿小不证明所有子步无运动或getter错误。
- 未验证：驱动、真实接触/求解语义及解析体校准；不能启动sleep或清速度制造通过。原文件入口R/runtime_repairs_20261011/round06/v179_analysis.json及C/outputs/fabrica_beam_loaded_v179_fixed_targets。

## S03 — 接触观察器构造有属性副作用，容量/根过滤假设已有反证

- 范围/版本：Isaac5.1匹配SDK与v173/v175，Beam支撑pair；没有物理修复因素。当前_get_contact_probe仍创建prepare_contact_sensors=True视图，首次构造会作者sleepThreshold=0/contact report threshold=0，后续缓存读取不同于构造。实际生产scene.get会重新包装SingleRigidPrim，不可笼统称其无副作用。
- 历史原证据：v175三边界native sleep阈值已为0，没有本轮0.01→0的变化观测；不能证明此属性是抖动/力缺口原因。睡眠体不报告接触，恢复sleep不能作为保留原力门的修复。
- 否定范围：v175 TGS/PGS各2880个支撑pair，active normal最大20/friction8；v173 normal34/friction9，容量64未满。normal明细重建matrix最大误差约2.1e-7/9.95e-7/4.71e-7N。当前板根实际为kinematic刚体、唯一mesh的normal已捕获，不支持容量耗尽或遗漏板大量shape解释当轮载荷缺口；不能泛化到全部夹爪/复杂未覆盖对象。
- 反例/未验证：net normal与已列normal一致并非独立力平衡；旧零normal pair未读取friction/阻尼/未知外部接触，不是完整账本。已安装Python源码和匹配文档没有证明“TGS只返回末子步力”，PGS同样存在缺口。详见原additional_native_review.md；本次保留文件与S01原始重算互相定位。
- 校准路线：0.25kg/6cm无SDF解析Cube落至解析静态薄盒平面，阻尼/sleep0、原CPU/TGS/240Hz/16-4/external=false，完整同帧normal+friction/COM/clock；measurement_calibration_preparation/experiment_contract.md预先记录自由落体1%、normal重建1e-5N、末秒冲量5%和原96门。只准备，尚未运行，不是抓取前置状态消融或五件成功证据。

## C07 — v185转向原点运动主要含绕接触区转动；H01帧漂移解释被否定

- 对象/阶段/版本/因素：柱3assembly_clearance，v185/31bb，继承局部Z=-4mm与L02；此次只读完成原始后审，没有改物理或保护门。
- 原始事实：18770模型锚点3.208063mm/本体固定校准锚点3.208172mm，方向变化.042849rad；帧变换平移漂移.247µm、转角1.71e-7rad。固定模型↔本体偏置没有解释超门，否定H01用于此轮，保留第一件旧反例。
- 接触/几何：14929原登记有载normal加权质心各5/6个有载点，冻结其payload材料坐标，在失败帧对应点相对移动0.095339/0.482862mm；CAD原点位移向量大部分由绕该区域转动项抵消。符合滚动/扭转的机制候选，不证明同一接触始终粘着、不称指垫滑了3.208mm，也不因此改变3mm/5deg资格。
- 控制/实际执行：原smooth转向1681条全published，最大joint速.274506rad/s、加速度4rad/s²、位置lead1.418mm/角lead.009133rad/关节lead.013011rad未超原限制；last_applied_actions和关节原样保留。全阶段native目标缓冲缺失仍按E01列未验证；get_pose body存在标记不是逐条native认证。
- 完整局部结果：pickup_clearance16225–17089实测世界Z+84.893227mm（替换此前partial31.583mm）；close原1N168、lift+35.234mm、peel+8.014mm与第一件5157参照仍保留。
- 下一可证伪因素：仅柱3转向angular speed .20→.10rad/s，原几何、力带、linear speed、joint speed/accel、超时、原锚点与门保持；若原3mm门仍在相同角进度触发且未减少材料点/原点运动，否定单纯角速度解释。不可由较长运行步数单独认定夹稳改善。
- 证据：round04_l02_full/v185_frame_postmortem.json、column_final_motion_and_material_points.json、completed_analysis.json、postmortem_raw/manifest.json。13原始子集及归档SHA核验；三原视频已取回、两端SHA核对。无完整装配/插入/释放/其他seed证据。

## V05 — L03独立服务器回归与部署

- 版本/因素/证据：edd661d；signed aggregate仅改support helper，加2反例；20本机、442服务器Beam unittest、237相关pytest通过，2 skipped。权威部署前861项零差异，部署后2文件SHA验证，原源码tar SHA72d7135ecda73146430f6bce4f9643e63a798d007bf7f80b73ca8ccfa689befc保留。
- 适用：落桌support sampler→valid→seat/watchdog/96与最终240共同门。负合力返回无效，真实seating反例不下降；正负混合按净方向，正向对照保持。
- 未验证：没有新原生运行/负合力事件；不解释v185原点转动或底梁raw速度。源码/测试/回退见round05_signed_support；原草稿与其20项证据不删除。

## 应替换的旧泛化

beam_grasp_handoff.md曾将滑动直接归因没有法向力。G05的v149双侧仍有力却滑移反驳其普适性；该旧文只适用于当时轮次。其他旧动力学/材料参数也须按版本核，不直接作为当前值。保留旧文件作为历史证据，不改写旧原始记录。

## 当前方法约束（历史恢复授权；第09轮后暂停，见M02/M03）

每轮聚焦当前瓶颈完整链，独立审查者先接收版本、原始观测和安全要求，后读旧解释；另一个视角检索历史成功/否定证据。全仓库广审在新瓶颈、跨模块变更、回归异常或连续两轮机制无改善时触发。用户2026-10-13最新授权：长仿真前尽量合并可独立验证、互不干扰的修改与结论；多个物理因素可能无法排除时保留对照，不把混合结果归因单个改动。每轮先审完整链、两轮无改善停线、阶段切换/跨模块广审仍保留；持续审查/修改/仿真累计上限20轮，不跨日清零。优化方法须用户确认后记录；此次直接指令即确认。

恢复后先建立成功底梁close/settle/短探抬锚点及柱3扫掠/双侧初接触对照；设计局部柱3原生实验减少长前缀，须对齐真实工装、材料、驱动、关节/物体状态与接近过程，并与全前缀核对，不能声称仅位姿/JSON能恢复求解器状态。局部通过后再全流程回归。若需证明G05因果，做当前成功几何下仅切换带载力调节的成对探抬；现无该消融结果。

## V06 — 慢转向已实际执行，但未改善柱3夹稳（v186完成）

- 版本de317d4，compile默认.20与显式.20完全相同，override.10只改柱3assembly_clearance角速；非有限/非正/大于.20输入拒绝。3本机、445服务器Beam、237相关pytest通过，2 skipped。默认161阶段/layout契约通过。
- 假设/支持/否定/停止先写在round06_turn_speed/experiment_contract.md；仅延长步数不作夹稳进步。保留原3mm/5deg/40N/96/240与原超时。补全阶段只读四边界诊断，v186于09:58:27启动、11:12:19结束，19789步exit0；实际task19788位置3.210373mm超过原3mm，方向.043013rad低于5deg，双侧8.568/7.331N严格接触true。仅位置门失效；不是缺双侧力。

- V06运行期配置核验：实际task_config_snapshot的161阶段与保留v184/IK替身编译快照只差柱3turn angular speed=.10；IK替身比较不验证实际路径。原生两机器人joint/command-target读数有效；空闲手指未缓存造成整体valid=false，要按字段/身体逐项统计，不以该总标志抹去有效数据或宣称全身体有效。原始配置比较在round06_turn_speed/actual_phase_contract_comparison.json。

- V06原生运行期第一件回归：5157条既有选定观测全部匹配，位置/TCP/q/双指力差0；原接触日志当前阶段之外和其他状态不在该证明内。完整比较round06_turn_speed/prefix_comparison_first_complete.json；v186已完成，仍不用第一件结果代替柱3转向。

- 完成后机制比较：实际published角速max.1000019995/median.0999998726rad/s，转向angle1.117239rad约64°处失败；旧v1851.173752rad约67°，更早超门，不支持“降角速即可夹稳”。模型/本体固定校准3.210373/3.210422mm，变换漂移.250µm，再次不支持帧漂移解释。源码862项前后不变。保留慢速选项默认.20与所有原日志，下轮恢复.20。原body/native字段覆盖逐项见completed_analysis.json，不能泛化为全身体/所有子步认证。证据round06_turn_speed/completed_analysis.json、frame_postmortem.json、turn_profile_after_failure.json。

- 下一预写可证伪路线：原4–12N是两指peak调节带，不是每指最低4N。第07轮拟只将柱3转向peak带提高到16–24N，原40N硬门、seek预算、命令lead/rate、位移/方向/96/240成功门保持；需要先验证配置确实进入实际publisher、实际q/力增大及相同转角下运动是否减小。输入限值不代表物理已验证安全或servo已可达。多因素归因仍受用户方法约束。

## V07 — 柱3显著提高转向peak预载候选（已测试部署，v187完成）

- 版本28f5dd9/第07轮；相对v185同速基准只把柱3turn peak调节4/12→16/24N，v186被否定慢速恢复.20。编译只增加一个phase一个字段，默认与显式4/12全阶段相同。复制spec保留capture、seek=.03、lead=.001、rate=.025、硬门40与原3mm/5deg/96/240。不是提高所有件/全阶段执行器力限值。
- 独立验证：11本机publisher/compiler反例（包括真实返回action、重复tick、24以上卸载/40以上原停止、缺测失败、错阶段/零件、seek/lead预算与trace），3 speed/10 relief回归；456服务器Beam与237相关pytest通过，2 skipped。首轮旧任务缺phase的兼容问题已在部署前修复，保留失败日志。权威部署前862项零差异、后3文件逐SHA及回退0e55707d614e74e0688d28d66dc43c93250fd1728bd1f5c1623dbffc2f56f68e。
- 原始反例/限制：v185/v186都有双侧力仍转动；低速不改善。实际drive max effort原生26与contact N不同，不据此推断夹持饱和；force_low/high作用peak而不是每侧最低值。高目标不保证能到达，输入边界不是安全认证。原native过载取消/卸载覆盖仍未验证。
- 支持/否定/未验证：实际力带消费/目标与真实q对账、持续96样本达到较高预载、相同转向角下root/material运动降低或越过旧1.17375rad才支持；过载/更早原门失败不支持。未消费/seek或servo不可达列无法验证，不放宽门/重置预算。v187已结束，原门下夹稳改善见V07完成证据；不称转向完成或完整装配。合同/审查/部署：round07_turn_force。

## E03 — v186全阶段采样动作目标与调用站点对账

- 范围：de317d4/v186，显式诊断PHASES=*、stride8、四边界；只读完成结果，不新增物理因素。各实际已运行阶段有四边界记录，阶段跳变少数标签计数相差1；joint SDK/native position目标最大差1.1919e-7rad，无缺目标。不是every-substep或全身体有效认证。
- 状态调用：记录15次collision=true及1次gravity=true，无已记录的关闭事件/位置速度强写；仅证明已插桩站点与所运行范围，未覆盖后续换手/释放及未插桩路径。原生有效性必须逐body/field读，空闲手指缓存缺测仍保留。
- 证据：round06_turn_speed/completed_analysis.json、evidence_validation.json、raw_windows/manifest.json；服务器18原jsonl SHA、取回19文件含三视频SHA核验；保留原失败和源码前后862零差异。

- V07启动后实际配置核验：161阶段，仅两个预写控制字段及派生recipe指纹改变；所有其他task字段相同。数据见actual_phase_contract_comparison.json，配置通过不证明增力实际执行。

## C08 — 柱3登记接触区与质心存在几何杠杆

- 对象/阶段/版本：柱3登记14929的v185/31bb normal加权质心材料坐标，与v186/de317d4同几何的原生COM字段。只读跨轮分析，无因素改变。
- 原始值：COM local[.0444746,0,.0386140]m；两登记normal质心均值[.0488535,-.0001515,.0621104]m，差长23.9014mm。支持检查偏心载荷/接触力矩候选，而不是从CAD root推断COM就在原点。
- 反例/未验证：材料点与COM跨轮且同源码几何，不是同步接触力矩测量；法向力质心不等于真实支撑中心，不证明持续粘着或重力是唯一原因。actual friction/未知机器人碰撞及contact distribution改变仍未验证。高预载运行本身需要实际对账，不改变原门。证据round07_turn_force/geometry_lever_review.json、v186 raw_windows/physics_boundary_trace及v185材料点文件。

## D01 — 不完整整爪/场景配对账本的独立诊断草案

- 原始范围：v185/v186整爪日志以场景对象为pair对手，且全局step去重导致登记后仅左手覆盖；其他机器人刚体对未逐项覆盖，零已知pair不是零未知接触证明。v186失败前画面可见两臂靠近，画面本身不证明撞臂。
- 候选因素/版本：28f5dd9源码冻结时，在独立diagnostic_net_proposal副本准备可选单次既有payload sensor API get_net_contact_forces，只在柱3turn after_physics读取。已安装5.1源码证明clone/dt转换及world/sensor方向；不新建view，未更改物理/成功判定。
- 本机证据：8反例含真实边界caller集成、37既有诊断回归通过，检查clone/dt、有效/缺测/非有限shape、无效handle不调用、sensor方向不反转/不加入pair总力。默认控制未部署变化。
- 反例/未验证：net与filtered共享backend，不独立认证力平衡；组件覆盖须解析校准，净差不能直接命名机器人碰撞。本机caller已验证；服务器/native仍待验证，不称完整碰撞覆盖。提案manifest/test_net.py留存。

- V07完整选定底梁5157观测：object/TCP位置、q、双指力最大差均0，未匹配的其他观测/状态不在这一证明内。实际pipeline预载尚待turn执行，不能称柱3已改善。

## D02 — 解析体校准分析器反例与准备纠正（历史准备；原生结果见V08）

- 独立副本neutral_calibration_revalidated，原Beam源码28f5dd9冻结。4合成控制：理想校准/原96门均通过；四分之一载荷与向下合力均拒绝；正确载荷但原生角速.07rad/s仍拒绝原门。不是仿真、接口校准或完整装配。
- 修复：analyze_box.py的numpy布尔值显式bool，原写报告失败被合成测试捕获并修正，保留旧脚本和失败输入。相同重力/质量/原96/raw门，非放宽阈值。
- 资产审查：已安装Isaac5.1 add_reference_to_stage默认prim_type=Xform，引用前DefinePrim；UsdObject两参数调用。原prepared根Cube受本地根typeName覆盖的风险，改校准资产为Xform根、Cube子节点及根MassAPI；离线组合已验证原0/fixed2有效Cube；原生仍待验证，不能泛化到有子Mesh的Beam资产。
- 判据仍按原先实验合同：0.25kg/.06m解析Cube、CPU/TGS/240Hz/16-4、no robot/SDF/控制器、g9.81/阻尼sleep0、自由落体1%/重建1e-5N/末秒冲量5%、原96共同门。准备源清单引用当前863；若后续源码漂移需更新或拒绝，不启动并行Isaac。

## V07完成 — 明显增力改善柱3转向夹稳，新停止来自底梁6单侧无力

- 28f5dd9/v187，19067步，13:07:51完成exit0、源码863前后零差异。柱3turn实际1.39334495rad=79.83°，比v1851.173752rad=67.25°、v1861.117239rad=64.0°更远；末Root2.041495mm/.0313976rad，双侧21.553/21.043N严格true，原3mm/5deg仍有效。新力带16/24已消费、actualq.716128/target.716051，仍40N硬门。构成局部夹稳进步，转向尚未完成。
- 原始停止：19066底梁6.state_sync contact_missing，左19.1234N/right0，两probe有效；Root.261119mm/angle.00721875rad均门内、无grace。不能称柱3又超门；不能由有地面支撑/Root稳定替代双侧资格。第一件5157 object/TCP位置/q/力选定对照差0；旋转分量补查已无差异，详报告取回后逐字段定范围。
- 反例/未验证：更强柱3并不保证底梁6持续保持、完整turn/首柱插入/五件。未知双臂/整爪接触仍需检查。C6 table relief配置在落桌后全部阶段存在，名义带恢复4/12不可归因配置缺失，须核metadata测量/force≤10/clock失效及真实arm target/execution。原newfailure所有日志/视频保留，不放宽contact门。
- 证据round07_turn_force/completed_analysis.json、failure_contacts.json、loaded/hold原始与prefix对照；后审和新完整检查点继续补齐。

- D02离线组合反证已完成：已安装SDK USD库按实际DefinePrim(Xform)+AddReference的路径，旧根Cube组合后两资产均Cube0；Xform根+Cube子节点后各1 Cube且CollisionAPI有效，box尺寸.06、质量.25、平面2×2×.02一致。纯USD无World/Physics/GPU，不是原生校准。原草稿不能再直接启动，替代准备见neutral_calibration_revalidated。

## S04 / V08 — 解析体参考通过，否定全局力/时钟/原门不可用泛化

- 28f5dd9源码未变，独立解析Cube/静态face、无机器人/SDF/控制器/阻尼/sleep，原CPU/TGS/240Hz/16-4/external=false。1009样本、46自由落体、接触后4秒，13:33:44完成exit0。native mass.25，g9.8100004；clock连续、自由落体max1.1215e-6相对误差、normal重建max5.722e-6N；末秒冲量相对残差4.678e-8、mean Fz2.452500N≈重量2.4525001。
- 原门：raw线速max.0001663m/s、角速max.001037rad/s，载荷/底隙/原raw速度联合最长960≥96。否定“相同TGS设置的接口统一只返回四分之一重力、时钟统一错误、原96门不可通过”的泛化用于此解析体。不能泛化为Beam资产/所有接触/完整机器人或五件通过，也不解释S01/S02。
- 证据neutral_calibration_revalidated/run_after_v187/probe全原始、observer/model/collider/clock与analysis；样本SHA862ae64b722ca689f76ff95a38c63d8877397afca8d2fb342a6564b5603d5eec。本机取回并SHA、源码863前后零差异。后续区分SDF/驱动/碰撞/阻尼等因素，不能清速度、启sleep、放宽原门制造通过。

## L04候选 — 高支撑负载时降压策略退出，保持原低压且不授予新资格

- 原始事件v187/28f5dd9 C6：19018support12.888N超helper的10N资格域，之前24真实资格已有，主动3/4与.0024恢复4/12与.025；19066support17.4847，pre左13.554/right.2536/target.6635015，post左19.1234/right0，原contact_missing拒绝；非Root超门/缺probe。配置一直存在，否定配置遗漏解释。
- 独立代码事实：helper将force>10作为invalid，active清0并恢复原carry；pressure_rate因此10.4倍，真实caller nominal卸载104urad而低压时10urad。只证明策略切换和时序，不证明其为唯一失去接触原因；未知碰撞与arm执行仍需查。
- 候选：先前已有资格且新的handle/运动/底隙/正力/连续时钟有效，遇高支撑力保留低压/慢调节；高于10不能新增资格计数，冷启动/时钟断裂/缺测/负力不保留。原40N、grasp、seating/96/240独立保持；无锚点重置。13本机relief含真实publisher递减反例、9net+37既有诊断通过；已部署abfc344，468服务器Beam与237相关pytest通过、2 skipped；原生已完成，结果见V09；此处保留预写判据。对应round09_supported_pressure。

- L04部署来源：abfc344，权威863项部署前零差异、4文件后SHA验证；回退aa699929c9462d34656d61a0592378829a49d6a2a996749f19725a0ea6b52edc。net诊断独立9反例与37旧诊断本机通过，只现有cached C3/C6/gripper sensor，不新增观察器；组件完整性/未知机器人接触/所有arm体尚待原生证据，不将净差直接命名撞臂。

## M02 — 本轮后暂停（用户最新授权）

- 2026-10-13用户要求“这一轮的流程结束后暂停”。第09轮v188完成后仅完成后审、回归、证据/回退和文档同步，不启动第10轮。此前最多20轮继续授权被本次暂停覆盖；后续物理修改与仿真需用户重新恢复。此条是授权状态，不是物理结论。

- V07后审收尾：完整姿态分量报告已取回，5157选定object/TCP位置与姿态分量、q、双指力最大差全0。原始窗口与三视频归档SHA9cbbba54d867023c55a6ffb4a821eb93712a654c89e34fe73709810d7ea3be46已核验。保留28f5dd9全863源码检查点SHA0448eafdbf0346a701040cd69544df3cf9e4877eb3d07cf0c000b3ef97bbe466；不把79.83°写成转向完成。

- 第09轮运行中回归：v188与v187实际task快照完全相同；第一件5157选定位置/姿态/q/双指力观测无差异。柱314950严格接触与保持有效；尚未检验高支撑负载修复。证据round09_supported_pressure/actual_phase_contract_comparison.json、prefix_comparison_stdout.json。

- L04原始停止反例（完整后审待）：v18819070，C6 pre18.269/.12702N→post21.713/.04458N，原.1接触门拒绝；实际仍3/4、.0024、资格24、卸载10urad。说明恢复4/12快速卸载不是此次失去接触的必要条件，不能把helper退出当唯一物理原因。柱3仍valid；精确角度与原门/源码/全诊断待完成报告，不新增仿真。

## V09 / C09 — 已资格高负载低压确实执行，但未解决底梁弱侧接触失效

- 对象/阶段/版本/因素：底梁6，在柱3assembly_clearance保持；abfc344/v188/第09轮，保留柱3depth-.004/turn.20/peak16,24，仅改变高正支撑负载时先前已资格的低压保持；另独立只读net诊断。实际task快照与v187完全相同；原门不变。
- 原生验证：15:15:23结束exit0，19071步；first高负载19018/table12.888317N，与v187原事件相同。43个高支撑样本10.1065–20.8453N实际采用3/4N、rate.0024，所有count24、不新增资格，真实target decrement每次10urad而非旧104urad。468 Beam/237相关测试通过、2 skipped只是部署前代码证据。
- 原始反例：19070 C6 state_sync contact_missing；pre左18.2693/right.127019N，control仍unload10urad；post左21.7133/right.044582N，两个probe有效，弱侧低于原.1N门。Root.258419mm、方向.007244rad、无grace，非位移超门/缺probe。此序列与源码peak-based卸载分支一致；证明弱侧接近原门时仍因强侧峰值卸载，不证明冻结/反向命令即可安全恢复。
- 物理效果：实际转角79.95652182°，v187 79.83278514°，仅多0.12373669°/4步；如实保留狭义旧角跨越，但不列实质夹稳进步或转向完成。柱3该段10228保持记录0 invalid/maxRoot2.067808mm/方向.032059rad；最后failed阶段柱3仍21.831/21.340N、Root2.049231mm有效。保持原3mm/5deg/40N和成功门，formal_success=false。
- 被否定/待验证：低压退出→恢复较快卸载不是此类失夹必要条件，修复这一切换并未解决当前瓶颈；不能说它从未影响轨迹/只有一个物理原因。后续恢复需重新比较弱侧余量、夹持几何、受载实际arm/gripper执行与非目标碰撞；不能继续盲放宽门或凭更多步数宣称改善。尚未验证完整turn/插入/释放/卸载240/五件，多seed也无。
- 回归与冻结：5157选定object/TCP位置、姿态分量、gripperq、双指力差0；源码864项before/after同SHA。四边界stride8所采样SDK/native目标最大差1.1919e-7rad，已记录状态调用16条；不是每子步/全部native字段或所有调用路径认证。
- 证据：round09_supported_pressure/final_chain_review.json、completed_analysis.json、pressure_postmortem.json、failure_contacts.json、actual_phase_contract_comparison.json、prefix_comparison_with_orientations.json；18原jsonl完整服务器保留，19关键窗口/结果/三视频本机SHA通过，归档SHAd7583de84ee285b04cde213022ad1a607999818256f55c3d9029d58a3639b087。原3文件回退与新增文件移除逻辑已对前版863项零差异，未执行回退；当前abfc保留为可复现未改善候选，实质进步完整检查点仍28f。

## E04 — v188现有20个sensor净读数可用，仍非完整碰撞/力平衡证明

- abfc344/v188，可选只读diagnostic，柱3turn after_physics/stride8，不新增view或改变控制；20个C3/C6/Robotiq sensor各247样本，全部valid，无重复/错误，19064最后采样早于19070失败。独立API组件覆盖未由本轮解析校准认证，不同方向账本不相加。
- 原始异常线索：左手left_inner_finger最后unfiltered_on_sensor[15.498,-56.147,63.537]N、norm约86.2N，已知C3 pair normal[20.775,2.363,-6.228]N；原始已知非零pair仅指定目标/工装，对手覆盖有缺口。这是读数差/覆盖线索，不能由差值直接定位撞臂、命名未知物理力或判定全scene力平衡；其他sensor零值不证明所有arm无碰撞。
- 证据/限制：pressure_postmortem.json、last_sample_nonzero_pairs.json、raw_windows/physics_boundary_trace.jsonl.gz；未覆盖所有arm刚体和失败当步，禁止把247 sampled变成全连续扫掠。需要恢复后独立核组件语义、完整pair/几何与实际执行，再作物理归因。

## M02完成 — 第09轮流程结束后暂停

- 本轮原生、后审、回归、回退/证据取回已完成；最终文档以documentation_sync.json的ready=true为SHA同步完成。第10轮未启动，无后台自动继续。先前20轮上限不是恢复授权；用户明确恢复后重新核源版本与失败链。

## R01 — 合并上游以来的汇报归因与证据边界（分析，不恢复实验）

- 版本/范围：2026-10-10合并d3ba9fa（上游父6f57dae），e6a保留反馈帧后与v169比较；至当前abfc344/v188，累计15个e6a后的修复提交、37代码/测试文件。工程改动数量不等于原生突破。当前服务器864项SHA复核一致，暂停、无活动原生worker。
- 合并回归：v169/v171同seed0/layout505都完成底梁抓取/脱槽/抬升/搬运，到严格setdown4800步超时、最长原稳定资格0；保住既有能力，不把相同失败边界当轨迹完全相同或落桌成功。合并原行为选择保留连续物理/自然释放路径，隔离演示固定附着/速度清零路径。证据outputs/beam_upstream_merge_2026-10-10/decisions.json、v171/merge_regression_comparison.json。
- 原生突破：柱3原抓位左力0、双侧资格0；仅局部Z-4mm后双侧≥1N/strict168、原96门通过，随后peel/lift/clearance。其几何切片宽度9.4318→12.0033mm提供候选依据，原生对照支持下移整体因素；不把宽度作为唯一子因。转向同速.20下peak4/12→16/24N，实际柱3约21/21N、67.25→79.83°，原3mm/5deg/40保护保留。物理证据G08/G09/V07；G01–G07更早底梁成功机制不能冒充合并后新突破。
- 探索的作用与限制：fca及v180–182以位姿窗口/向上支撑/连续时钟和失抓guard有限推进到第二件，formal_success始终false；原raw速度/负载严格共同门未通过。这是隔离后续瓶颈的研究通路，不是修复严格落桌。v182_analysis.json原始2456落桌观测原共同max0/96。
- 中间操作：按行为整合上游；实测q不得由命令缓存代替；修正实际apply_action日志/native目标对账；全条件同帧连续计数、共享抓持/插入契约；普通减压受限/时钟断裂重资格；planned与issued回中生命周期、确认窗口peak、signed净承重；几何4mm与同速增力；慢转、PGS与固定目标反例；独立解析Cube校准。源码/反例与所到达原生阶段分开记录，未来共享换手/插入/取消分支尚未全面原生覆盖。
- 最新直接失败/未决根因（以下为碰撞空间定位前的原始后审；当前升级结论见C10，优先避让）：v18819070 C6强21.713/弱.044582N，弱侧低于.1门、probe有效、Root.2584mm；C3仍有效。pre18.269/.127019时因peak过4仍卸载10urad，post弱侧掉门，实际高负载分支43样本一直3/4/.0024/count24。仅比v187多4步/.123737°。已确认控制与反馈序列，未证明卸载是唯一物理根因；受载几何、实际动态执行与非目标接触仍须比较，20sensor净差不能直接归因撞臂。见V09/C09/E04。
- 汇报验收范围：最新468 Beam/237相关pytest通过2skipped、第一件5157选定观测差0、源码前后864不变；代码质量与证据可信度提升，不认证装配。严格落桌、turn完成、插入/换手/自然释放、最终240共同资格、五件与多seed都仍未完成。解析Cube正常参考只排除全局不可用泛化，不排除BeamSDF/drive/复杂接触问题。保持第09轮后暂停，不开启第10轮。

## C10 — v188左指尖与右腕2几何冲突及先于底梁失夹的受载链

- 对象/阶段/版本/因素：abfc344/v188，柱3assembly_clearance，左Robotiq left_inner_finger指尖与右UR5e wrist_2_link；只读旧native关节/刚体姿态、USD实例碰撞网格与视频，不改变任何物理因素或新增原生运行。用户从视频提出撞臂，触发空间定位后审。
- 空间证据：用已安装UR5e URDF与记录的原生q重建右臂；两臂wrist3位置与native gripper fixed mount回读最大差.392/.335µm，USD零位关节位置与URDF差<.067µm。以原生左内指姿态变换真实已启用convexHull指垫；19000有11个右wrist2表面顶点落在左指尖凸碰撞体内，最大记录面内余量.513936mm（19032）。机器人右腕碰撞为convexDecomposition，未重建cooked子块，不冒充原生碰撞pair冲量。原native metadata author_gripper_collision_pads=false，未将运行新Cube混为原指垫。
- 时序：18992未见顶点内含且同sensor净normal与目标normal差0；19000重叠.325mm，额外读数向量差3.54N；19008重叠.476mm、差44.22N；19018 C6support12.888N；19032最大.514mm；19064左sensor norm86.20N/目标normal21.82N、向量差91.21N；19070底梁右.044582N低原.1门、左21.713N，C3仍valid。19064顶点测试无内含见证不等于mesh已全分离，也不据净差直接认证力平衡。
- 右臂受载反馈：对应区间published arm target六分量零改变；FK腕2相对target误差由.5199→.6887mm，方向变化与额外接触出现同步。是原生q/target+已校核FK的推导，不是腕2body原生pose直接采样。
- 结论升级/限制：空间、视频与载荷时序共同支持非目标指尖—右腕冲突为底梁偏载失夹的首要物理诱因；L04压力切换并非最上游解释。尚无该具体pair原生冲量或无碰撞避让对照，不宣布唯一必要根因，也不据顶点检测未命中排除柱3本体的其他接触。之前C09/E04的未定位状态被此定位证据更新，原历史保留。
- 证据：round09_supported_pressure/collision_review/mesh_overlap_audit.json、authored_collision_meshes.json、arm_pose_reconstruction.json、collision_force_timeline.json、right_arm_load_response.json、视频front70/75/78/79/79.3和left_wrist79.3原片段帧及review_manifest.json。

## U02 — 每轮自动审查漏掉实际指尖/持件跨臂碰撞的覆盖缺陷

- 原生报告v188 runtime monitor595检查/0collision，仅简化link-origin胶囊；refresh_environment_from_task明确跳过attached_to/grasped_by，持件未作为随动几何检测其他机器人。20sensor净读数和9link场景pair不覆盖所有右臂刚体，因此零已知pair不清除跨臂碰撞。
- Stage precheck为passive、stride128/4waypoints，只覆盖简化URDF骨架与未持件障碍；basename映射把Gripper/.../base_link降为URDF机器人base_link。真实helper+installedURDF frame列表离线反例误映射.688m并缺8个夹爪frame；native当轮runtime frame列表未保存，不把静态反例写成已读取原生预测坐标。588agent_env候选不是全实体扫掠通过。
- 后审脚本analyze_native.py10投影result时未纳入runtime_constraint_monitor/stage_trajectory_precheck；不重建碰撞网格/载物扫掠。此前只核正面与右腕末帧，未逐段审左腕关键片段；v187列未知跨臂接触后仍优先压力试验，v188额外净力又只留未决而未空间定位。此为审查优先级和完成范围的遗漏，不能归咎用户视频或说数据证明没有撞臂。
- 固定方法已获用户2026-10-13确认：仿真前以全部真实夹爪/持件与另一臂的扫掠几何及覆盖清单作固定检查；异常净力、实际跟踪偏差和三视角关键区间必须对齐；缺测/未建模不得用0collision作为安全证据。详见M03与根目录BEAM_REVIEW_CHECKLIST.md。本次只更新固定流程与交接；检测实现、轨迹和控制未改，实验仍第09轮后暂停。


## M03 — 用户确认实体扫掠与三视角时序复核加入每轮固定流程

- 适用/生效：2026-10-13用户明确要求“帮我把新的检查加入固定流程，做好交接”。适用于本项目后续每轮分析、修改和实验；本次仅方法/文档生效，不恢复第10轮。根目录AGENTS.md与BEAM_REVIEW_CHECKLIST.md为固定入口。
- 新增检查：双方机械臂全部相关实体、实际夹爪指尖/指垫、随动持件的轨迹扫掠；唯一完整路径坐标映射与原生姿态校核；目标夹持只精确豁免特定对象对，不整体忽略持件。已命令与已实际执行的基线轨迹都核，记录覆盖/错误/跳过、冲突对象对、时刻、最小间隙和缺测。
- 后审要求：保留并审查stage_trajectory_precheck/runtime_constraint_monitor原报告；正面、左腕、右腕关键段与同物理step的目标/实际关节、目标接触、非目标接触及载荷对齐。异常净力/跟踪偏差先空间定位，再决定调力/控制路线，不仅看末帧。
- 依据/反例：C10实体几何及载荷时序与U02的0 collision报告直接冲突；简化胶囊、持件跳过、base_link别名、缺8夹爪frame、漏汇总字段及漏看左腕关键段导致覆盖不全。读数有效或0已知pair不能认证全实体安全；顶点内含未命中不能认证mesh全分离。
- 保留方法：代码/原始证据先于旧解释；完整瓶颈链；长仿真前合并可独立验证的无干扰修复，多物理因素保留对照；两轮无实质改善停线、阶段切换/跨模块广审；局部通过后严格完整装配；不改物理/保护门制造成功；所有机制与否定路线保留。
- 未实现/未验证：本次未修runtime_monitor、预检查frame映射/网格覆盖、自动汇总程序或避让轨迹；未获取特定pair原生冲量、无碰撞对照，未新增仿真。固定流程已确认不代表工具检测能力已经补齐。下一对话优先补实现并离线验证，再根据用户恢复指令实验。
- 当前版本复核：服务器abfc344的864源码项逐SHA零差异，轮次表paused=true/completed=9/active=null/next=null。未发现C下本项目运行worker；服务器另有PhyAgentOS进程，未操作，恢复前重查资源。

## S05 — 严格落桌与探索放行、速度判据的范围复核

- 适用/版本：历史e6/fca、v182与当前abfc；2026-10-13仅代码/既有证据分析，无新物理改变。原落桌要求向上支撑1.962–40N、底隙[-.5,1]mm、raw线速<=2mm/s/角速<=.03rad/s共同96步。fca起探索模式以actor/COM双位姿窗口、路径/最大单步/端点速度、向上承重和固定锚点等独立门有限放行柱3，formal_success=false；不能以“原门代码保留”说实际放行条件没有变化。
- 反例：v182探索96资格时原raw线速5.018–7.111mm/s、角速.4147–.7133rad/s，原共同max0；只支持探索位姿/承重判据成立，严格落桌未通过。后续postseat guard不要求持续原raw稳定，首柱保持后即止。最终240由5fe要求全部条件共同连续（更严格），edd拒绝向下净支撑。
- 判据边界：PhysX求解状态的速度与最终采样位姿差分需同body/COM、坐标、step对账；位姿小不能直接排除步内震荡或动力学问题。解析Cube原门最长960仅排除全局统一不可用，不认证BeamSDF/drive。原门是否误拒/误放尚未完成复杂接触对照，不预设SDK bug或盲放宽。既有PGS/固定目标失败结果保留。
- 原生/项目划分：位姿、六维速度、配对接触、时钟来自SDK；对象筛选/聚合、底隙、相对滑移、有效性/连续计数及阶段成功为项目处理，不称Isaac原生装配成功。main2739的abs净支撑与当前signed修复要按提交区分。
- 证据/未验证：C01/R01/S01/S02/E03；outputs/native_gate_review_20261013、seat_gate_transition_review_20261013、velocity_gate_validity_review_20261013及完整旧交接归档resume_20261013/handoff_before_fixed_collision_checks_20261013.md。唯一速度差来源、Beam无机器人/夹爪保持对照及完整接触力账本仍未验证。

## T01 — 长仿真成本与本周汇报口径

- 仅查询既有起止：v183为44分30秒、v184为47分20秒、v185为66分08秒、v186为73分52秒、v187为72分24秒、v188为72分37秒；最近三轮均值72分58秒。口径为启动至进程退出，不含修改/检查/复盘，不证明完整装配耗时。原始起止与证据入口保存在上述旧交接归档及各轮completed文件。
- 本周教师/领导汇报一页：局部抓取突破、同速增力转角约67°至约80°、实体干涉瓶颈、优先避让/分段验证。保留完整五件及严格放置未完成的边界；不以15提交/37文件或测试数当物理成功。PPT路径outputs/beam_weekly_report_20261013/deliverables/Beam修复本周进展_20261013_定稿.pptx；原始依据见R01/G08/G09/V07/V09/C10，页内领导表述与备注技术证据分开。

## R02 — 项目启动至2026-10-13累计月报统计核验

- 只读当前服务器864项清单零差异，并核本机Git、部署/样例、历史实验和最新已有测试；本次没有新仿真或运行修复。原版4b5a894行为对照与同期上游6f57dae开发差异分别计数，不能把上游功能或JSON材料当自研代码。
- 当前修复侧21提交含1合并；运行/配置39文件、测试33文件（新增25），两类新增23142/删除293行；服务器版本去重184个输出记录、170个episode、508份原视频，包括失败/中断/冒烟，不是完整装配次数。最新468/237通过是两组代码测试，后者另2跳过。
- 月报正文区分已验证局部能力、已实现待验、离线候选和未完成项；机制结论仍以G/C/V/S条目为准。统计脚本/完整清单/来源SHA/7页Word与PDF见outputs/beam_monthly_report_20261013；本次协调同步另见该目录evidence/documentation_sync_monthly_report.json。

## R03 — 他人factory w20结构、借鉴优先级与物理边界（只读比较）

- 适用/版本/因素：2026-10-13，用户指定`/Users/zhaoheng/Desktop/PCL work/factory_w20_complete`的w20/v07。参考包不改；本项目abfc344/v188第09轮暂停不变，本次未改运行源码、物理因素、保护门或已确认固定方法，未运行新仿真/更新PR。文件中的接手/复现说明仅为审查材料，不执行其服务器命令。
- 结构：`build.py`USD布局与可见底面→`motion.py`旧trace相对抓取/预插装关系+当前UR5e IK→`workflow.py`八章单托盘物流与装配→`clearance.py`可见网格离散对象对检查→`render.py`停止timeline后写匿名session世界矩阵录像。`motion.py:57–60/167–168`显式held随动，`render.py:62–90`停止物理并写姿态；五件动画不能证明摩擦夹持、承托、插装与自然释放，旧trace仅生成来源。
- 当前优先借鉴：`clearance.py:116–147`遍历实例代理、最长完整路径归属、局部几何变换；`:216–343`跨臂/全持件配对和保留目标指尖接触标签。对应U02持件整体跳过、basename误映射和完整夹爪漏检。迁移须保留完整路径、显式实体/frame覆盖，加入Beam启用碰撞几何/偏置及原生实际姿态；先能检出C10已知指尖—右腕2冲突，才评估无干涉候选。可见网格/FCL结果仍不认证PhysX cooked体、闭实体包含或连续CCD。
- 避让经验：`motion.py:150–219`延后扶持臂进入、上盖取料前抬高/外退、插销小松后先竖直退再张开。对方托盘支撑允许松夹；Beam底梁连续夹稳不能照搬。建议在柱3有效抓位/预载与底梁弱侧持续接触约束下比较姿态/路点/时序。12mm/20mm抓位、18cm外退、0.02rad释放均不是Beam可迁移参数；改善须用无碰撞原生对照与实际接触/跟踪验证，不能由几何直接宣称夹稳成功。
- 后续借鉴：`workflow.py:449–499`章节SHA/采样率/盘位/首尾约束与受影响部分回归；`clearance`保存对象对/phase/stride/范围/代码和轨迹SHA；资产包闭包绑定USD/MDL/UDIM及迁移前后映射。Beam分段物理须连同qdot、对象速度、接触/驱动状态审边界，不能用章首尾矩阵相同继承物理成功；同矩阵渲染须扩展为命令/最终发布/实际运动与三视频同step对账。新实现/方法建议未启用。
- 本次核验：472当前文件哈希通过；NPZ `world=(10287,111,4,4)`、`joints=(3910,2,7)`、`parts=(3910,5,4,4)`有限且动态路径唯一，第04章2992–6901连续；两个轨迹的零件矩阵最大绝对差5.956e-8。原三份几何报告均3910帧stride1，零件5×38、跨臂19×19、环境相关1697表面，原报告非预期相交0；本次只读报告，未重跑FCL/USD/PhysX，仅查看提供的3张关键帧，未审完整视频。包内没有w16原NPZ，6377不变帧仅核代码与旧报告。
- 来源变更：原几何报告workflow SHA与当前相符，scene/audit-script旧SHA不相符；核core_sources.zip差异仅clearance移除服务器FCL路径、render改独立临时Kit缓存。迁移原报告声明224层路径归一化内容一致、178组合层/554引用通过，属于对方交付证据；本次仅472哈希，不改写成当前全依赖或几何重验。
- 当前权威核验/证据：服务器manifest仍`51dab343...`、repair_commit abfc344、864项逐SHA无差异；L864/W853匹配，11既有非运行文件缺W；paused=true/completed9/active-next=null，无C实验worker。`outputs/factory_w20_comparison_20261013/evidence/{beam_authority,local_review,package_integrity_current,documentation_sync}.json`保存本次证据；协调文档/证据同步状态以新documentation_sync.ready与逐SHA为准。

## M04 — 已授权恢复与干涉优先级

- 用户2026-10-13明确继续修复，按照既定工作流，上限20轮；承接此前9轮，不跨日或新对话清零。
- 异常载荷/跟踪偏差先排查双方实体和随动持件的空间接触，当前优先C10/U02；保持已有柱3抓位/预载与全部原成功/保护门。左右臂轨迹避让均纳入筛选，不以只改力度替代已知干涉处理。
- 这是既定M03检查的执行与根因优先级更新，不新增未经确认的方法。原生避让对照完成前仍不称唯一根因成立。

## C11 / U03 — 完整实体检测修复与旧冲突复现（第10轮离线）

- 对象/版本：旧abfc/v188实际全路径；新5e295f0检测器，物理轨迹暂未改变。48启用碰撞体、32机器人刚体、5零件及显式环境；完整2384步采样，9个跨臂重叠均在19000起的首柱转向。凸包包络包含法/距离复现原指尖—右腕冲突，支持C10但不补齐具体pair原生冲量。
- 修复：唯一full-path frame/原生body绑定，禁止同名base_link映射；持件始终参与，豁免只到自家实际inner finger接触；保持臂也参与预检查。原报告保留、全DOF名、逐pair最小值和实时原生body位姿流；缺测、simulation-selected偏置、包络/cooked/CCD界限分列。
- 验证：469 Beam/244相关pytest+2既有skip；最终full-path反例及62约束测试通过。原生监视器64步检查进行中，未认证物理避让/抓稳。原始候选源码9文件逐SHA部署866项匹配，原864项回退归档保留。
- 新反例：安装PhysX Schema接触偏置默认-inf（随shape extent由仿真选）；SDK GeometryPrim getter还是Schema读取且可能Apply API，不提供实际选值。非有限offset必须unknown，不能当负margin；新包含/旧跨臂反例防止假零检测。
- 离线候选：理想刚性持件先抬高再转向下降，+4/+6cm冲突，+8cm只有3.07mm，+10cm右腕—持件最近22.43mm；+12/+16cm等无包络重叠，完整变化路径最近24.87mm为自家knuckle—持件固定间距。采样<=2mm/0.005rad；右持件及完整右臂保持在检查。此理想路径不是已发布命令，未修改轨迹/新长仿真。
- 误差与边界：USD关节q对应从左原生姿态推导，最大角差.001071rad、位置差.2167mm；右原生inner finger也校核，未认证所有子步或原生cooked submesh。固定底座与桌面已有安装重叠保留原报告，筛变更路径时单列不动底座，不能命名新运动碰撞。未来DOF名直接记录。

### C11 / U03 原生可用性完成与命令路径修正

- v190/8d0fe63，64步原生只读复验：48碰撞体、8检查，原监视/预检查实体初始化/读取/live-handle错误0；两臂各12DOF名与推导顺序一致。相对首v189短检查q差最大1.93e-10rad，4个VisualCube腿明确无启用碰撞。原生初始三视角已审；不证明携件/转向或所有子步安全。
- SDK SingleRigid/SingleXForm默认reset_xform_properties=True会整理属性。新绑定显式False；仅复用live原生句柄/精确fullpath，无新增contact sensor或运动setter。此前v189不是严格无属性重写诊断，保留原始，改后v190为当前基线。
- 先前物体原点线性插值不是实际smooth publisher共享TCP插值；原结果归档不再作当前路径依据。按源码共享TCP linear/slerp复核：未避让完整未来路径仍有右腕—持件包络冲突，+8cm最近23.93mm，+12/+16cm无包络冲突，最小24.87mm为自家knuckle—持件不变间距。速率限制/跟踪/滑移/真实contactoffset仍待原生。
- 第11轮+16cm候选：473 Beam/245相关pytest+2既有skip；真实编译只改2目标Z，fingerprint为正常派生改变，其余非fingerprint字段/161phase全部相同。尚未原生，不称夹稳改善。

## V10 — 柱3高处转向避让候选（第11轮受控停止，后续见V11）

- ec08ec7/v191：只改变part3 pickup/assembly clearance世界Z各+0.16m，保留原hover/插入目标、抓位depth-.004、turn.20、peak16,24、原3mm/5deg/40N/双侧/96/240与探索守卫。右臂和底梁仍真实夹持并全路径参与检查。
- 473 Beam/245相关pytest+2既有skip通过；实际编译161phase及非fingerprint其他配方字段全相同，目标修改正常改变recipe_fingerprint。第11轮在底梁前缀受控停止、无避让物理结果；后续见U04/V11。
- 数据/回退：round11_raised_turn/experiment_contract、source_manifest867项、deployment_verification；source_before_raised_turn完整866项回退保留。此前物体原点插值不是publisher路径，当前依据TCP共享插值+完整实体包络，实际速率限制/跟踪/滑移仍需原生核验。

## U04 — 新碰撞观察器不得调用带资格副作用的公共状态getter

- v191/ec08：新增pair分类每次get_tracked_object_states，该接口调用_physical_hold_valid并更新hold_validation_stage/current_contact_metrics/hold_result；实体pose和持件预测也调用。原64步无持件检查只验证可读性，未认证此语义。每8步约1.7–2s，源码独立复核确认getter副作用。主动在底梁前缀停止，未到柱3，不能否定+16cm路线或计两轮夹稳无改善。
- SIGINT后exit0但最终episode/monitor汇总未导出，结果数组为空；保留原始流、三视频和源前后SHA，明确报告缺失。原始部分证据可追溯，不冒充物理失夹或成功。
- 第12轮修复：从既有native rigid wrapper直接get_pose；holder只读attachments/shared map；一次不可变context/holder快照分所有pair，legacy盒/阶段持件预测同样绕开资格getter。共享持件只精确豁免各登记holder inner finger；native close_and_attach亦按明确目标/robot/finger豁免。原成功门、目标与stride/覆盖不改。
- 独立反例：资格getter抛错trap、attachment深拷贝前后相同、generic多pair仅一次getter、真实对象pose直接reader、native close仅own target finger允许；473 Beam/249相关+2既有skip、最终67约束通过。实际持件阶段的运行期0误调用及性能/几何/控制结果待原生复验。

### U04 第12轮部署与运行期合同

- a2a37d5/v192已部署并启动，867源逐SHA匹配；同+16cm物理目标，原success/force/slip门/stride不变。worker运行期栈guard在碰撞integration调用公共资格getter前阻断，记录误调用数和已观察持件phase。当前无物理结果。
- v191最后raw3328而最终episode报告缺失，不能当最终steps或错误0；416实体样本至最后记录无stride缺口。受控中止部分证据16MB归档SHA833cf466d2492660e1636b6f9aaae9b96e4154ebfe724c0e49a11d27ad135e20，完整raw仍在服务器。

U04运行中补充：v192第一件5157既有选定观测全部匹配，初始到落桌探索的位置/姿态/TCP/q/双侧力无差异；外部getter误调用数0覆盖已执行底梁持件phase。未认证柱3新路径或严格落桌。

## V11 — v192原抬升滑移，避让段尚未消费

- a2a37d5/v192，15919步；双侧168资格和脱槽通过，15918柱3相对位置3.003mm>原3mm、姿态.047rad与双侧8.64/7.69N有效，底梁有效。原2个Z+.16m运输目标尚未执行；不把更早失败归因为避让高度或宣称干涉已解决。第一件5157选定观测0差异；observer持件全过程误调用0，raw报告errors0，source867前后不变。
- 空间先审：六时刻三视角同step，114完整实体采样跨臂/持件对另一臂最近128.18mm无重叠；108既有原生pair probe选定时刻有效，柱3非目标pair均零载荷。fixture凸包候选为几何包络，与原生0力/实际脱槽视图区分。采样/CCD/cooked/所有pair缺测保留；并不否定C10旧turn冲突。
- 新反例：第一件选定观测一致不能保证柱3微扰也一致；接近前柱件微小姿态差、实际闭合轨迹/载荷变化，旧v188 lift到16177而本次到15918。高处路径未执行，不能用这次停止评价原计划的空间因果对照。
- 第13轮a8e90cf：原限值内提早16–24N peak带，仅柱3lift/postlift/pickup3phase。479/250+2skip，真实编译3metadata、同目标，6闭合状态x775完整轨迹包络无重叠/min24.385mm。对照以实际抬升余量和完整turn/双件持续保持判定；若无实质改善停线广审。
- 证据：round12_observer_snapshot/completed_analysis、failure_review，7.7MB三视角对齐后审包SHA73e67233cbc565020f06fea1d341f6447e53d8aec3d05dd3cd5184edfdcb51b9；原1GB接触及所有raw/3视频有retained_raw_manifest，不删。第13轮contract/geometry/tests保留，尚未物理验证。

V11未决接触几何补充：v192/15912原生目标pair承载点加权中心part-local X错位5.266mm、Z1.150mm，柱3authored尺寸X11.50/Y12.00/Z76.20mm。近±Y的对置法向在偏斜位置加载，可能促成滚动；较高预载未必改善，也可能放大偏载。仅瞬时分布，未完整力矩/惯量/全部pair对账，不宣称根因；第13轮作为受限实际对照，无实质改善则停预载路线。证据round12 failure_review/contact_distribution_review.json及hypothesis.json，由原始过滤pair接触点/force独立计算。

V11后段预检查范围补充：首柱10原插入target、148有效名义TCP/IK采样，6闭合状态x148，跨臂/持件对另一臂最小42.787mm；末端桌面/支撑各-53.3nm接触候选及对象配合凸包候选均保留，未报机械臂重叠。仅名义endpoint/chord，真实反馈servo未替代；原工具0采样/IK失败结果归档并修正，不作安全结论。服务器round13 insertion_geometry_screen/insertion_gripper_range_screen与原报告保留。

V11/v193运行中复验：第一件5157选定观测0差异；10950探索放行标记明确original_seat_ready=false/exploration_used=true/formal_success=false，严格落桌仍未过。已到柱3descend，未到预载/新避让；原生实际结果待完成。证据round13 prefix_comparison_with_orientations/first_exploration_capture。

V11/v193抬升进展：16361进入pickup_clearance，原lift/postlift已通过，滑移2.298mm/姿态.0356rad、双侧20.8/19.9N/底梁有效。实际publisher消费16–24N且实测gripper q约.71627；改善相对v192的lift失败，旧v188也曾lift过，不能扩大成完整避让/装配。高处目标实际扫掠/完整turn及后续仍待验，最终原报告/源after尚未输出。

V11/v193原转向阶段通过：已完成+16cm pickup与assembly_clearance并回原hover（24216），两件持续真实保持，柱3滑移2.248mm/约6.8N双侧（hover回原4–12N），底梁有效。75–85deg窗口28个native实体姿态样本最小跨臂/他臂持件98.917mm，旧指尖—右wrist2在80.163deg处147.510mm；native79.856deg snapshot18夹爪sensor法向净差0。支持避开旧干涉/保持改善，尚非唯一因果证明/全力平衡/CCD或正式五件。实际插入与三视角/全轨迹/原报告待审。

## V12 — 末段插入完成容差大于已安装接口保护

- a8e90cf/v193：25711失败，source868不变，原lift/+16cm pickup/179.909deg turn/hover通过，双侧168/全程两件保持有效。3213完整实体样本无缺步，抓后1338跨臂/他臂持件min42.641mm/重叠0，原报告错误0；三视角与q/发布/接触/载荷逐段审，raw及10.04MB后审包SHAe2d617bee522f2a13b855768617056e5fe03314c7c9616e3a202d5988e56cd64保留。
- 最后insert_09以TCP6mm/对象8mm（600后15mm）准许complete；进入physical_hold_check(step0) installed3的3mm/.05rad门拒绝pe5.509mm/ae.002695rad，pair读数0N。base目标误差.185mm、part3主要悬高5.5mm，共同CAD参考目标完全同；不是新增跨臂冲突/失夹，也未完整首插入。
- cbfd89f第14轮：仅4个final_insert六field收紧1.5mm/25mrad，原3mm/50mrad及握持/40N/96/240均保留，继续原同target反馈servo，不扩大安装门。484/250+2skip，5.509mm实际完成反例与600step禁止再放行，真实recipe除4x6字段及派生fingerprint都一致。原冻结v169其余控制/layout仍digest验证。
- 后段native/实际接口仍待v194；只记已到达新门槛，不把turn/局部保持/名义几何或explore标记当正式五件/自然释放。原SDK环境漏载一次collection error已纠正，原候选与本机源/清单必须部署后逐SHA核对。

V12/v194前缀复验：既有5157个contact观测(taskstep1309–10950)位置/姿态/TCP/q/双侧力0差异；observer getter误调用0，10950仍是探索放行/原严格seat false。仅认证底梁前缀，柱3与新的final_insert待本轮实测。

## E04 — 任务步标签不能替代视频与四边界的物理时钟

- 独立source核验runner在after_observation后is_done递增taskstep，视频及geometry用递增后标签。普通frame S(index S//8-1)物理clock与before_action cycle S一致，after_action同clock发布下一间隔目标；同标签after_physics S晚1dt=4.1667ms。旧v193选after_physics同taskstep并称三视角同刻的陈述撤回，原raw/旧包保留，阶段结果/几何独立记录仍有效。
- 8时刻重取same-clock before_action/filtered contacts和after_action，following feedback另存：两臂motion/native q差0，88个同pathbody直接对照位置差0/角差max5.97e-8rad/错误0；每时刻11body直接复核，不宣称独立复核全部44。几何无自身clock字段，来源顺序与native姿态吻合共同约束。终端重复step/额外frame单列。
- after_physics sensor法向净差原结论只属于其自身物理clock，不拿作该frame同刻全力平衡；before_action当前未存unfiltered净量，明确缺测。属于既有同物理时刻检查的bug修正，不改变验收方法、物理/控制/原门。证据round14_strict_insert/align_keyframes.py、round13_clock_reaudit/physical_clock_alignment.json及原源码调用顺序。

V12/v194完成：25854步首插入探索停止，original hold_check96通过，原installed3mm/.05rad在末刻pe.280187mm/ae.002673rad；两件全程hold有效/严格双侧各168、179.909deg turn通过。source869前后一致/exit0。3231实体样本无缺口，close后1355 cross/他臂持件min42.6367mm/重叠0；原runtime3231/stage404无error/skip。真实配置24完成字段+fingerprint，104731同key contacts首数值分歧25703 final09，而此前全同；old failed/new继续insert。支持预算修复作用在末段。原3→6normal/friction仍valid0N，不能称已证明承重配合/释放/五件；original strict seat false。20.53MB包SHA dc689d60968bc75560a8bce1f076f15979e0a8acf1be411f3d6792d52a224cc1；3视角8时刻已看/同clock qdiff0及176同pathbody pdiff0，影像renderingTime未记录仅源cadence推断、current-frame unfiltered缺测按E04明确。保留cbfd89f候选/全部原资料。

## E05 — 渲染ReferenceTime与物理时间有不同起点，必须记录而非按标签替代

- 93ba920/v196/128初始步：Replicator1.12.27原ReferenceTime关联同render product，三视角各16帧/48有效记录，间隔1/30s，physics−render约−.820833秒，三相机一致、短窗offset变化26ns。不能据数值不同叫延迟/错帧；保留不同clock和offset，不自动认证绝对freshness或长窗映射。
- 原v194前128步64四边界：实际q、native目标和直接有效body位置均0差异；871源前后同、exit0。仅初始可读性/行为范围，不证明持件passivity/原seat/turn或装配。turn既有cached unfiltered法向净量现4边界保留，原生持件范围待后续。
- v195记录器缺os导入、没有clock输出但exit0：该例否定“正常退出=验证通过”。补导入与实际写帧开/关2反例后491Beam/252相关+2skip通过；旧边界断言/错误fixture修复和脚本后置文件名错误均保留，未改目标/控制/原门。全部SDK物理控制源SHA同v194。
- 2.35MB包SHA3520d53ba783f0ab8c6372baef9f6bf4d144c452ee9c2f75de9c1fcff175bf41；回退cbfd869(8bebb60...)和0a58诊断871(fa178934...)保留。适用当前SDK/opt-in flag，数据不可用保留unknown，未额外步进render/physics。

## C12 — 后段右臂侧抓的关节分支与环境干涉（第16轮名义）

- 431050c/872 opt-in高位分段：完整双方32robot body/全夹爪/5part及启用环境48collider，v194最终44姿态/物理25851为共同起点；左整臂和柱3不跳过。原退避含起点min42.720mm，外退备选更差。当前严格seat/自然释放及新后段原生尚未通过。
- 独立最终命令链查到prealign标.035但最终.004rad，240步仅走.96rad仍差2.1725rad。按该末状态的warm FK/IK名义路径腕1/前臂对桌面/缓冲台冲突；原静态终点全达假设撤回，原raw保持。去掉预对齐无改善；2个清晰IK端点的直接转换有大量环境/左夹爪重叠，拒绝。
- 新两joint阶段：原高位后wrist2至+1.50（避pi wrap），实际joint发布上限.004rad，实际关节误差.005/连续8才完成，240步不能提前放行；left3保持/原目标/原保护门同。真实编译只2阶段/prealign_steps及派生max_steps/fingerprint，494/252+2skip通过。
- 3384名义样本2姿态插值，无新增非目标重叠，右动体对环境min17.044mm；81闭合状态只标右inner fingers与新柱3目标接触，非目标min10.659mm（左指尖—右指尖）。固定安装/左目标/fixture凸包与配合原候选保留，不报整个场景0collision。两臂实体/持件都核验，没新增力修复来掩盖干涉。
- 范围/反例：FCL负距非native穿透深度；默认offset/native cooked/CCD/self拓扑/实际反馈及动释放/持件受载未证。source回退9c99910...及93/物理cbfd均保留。不能把名义端点清晰、通过回归或新阶段存在当实际换手/正式五件通过。

## S06 — 第17轮提高底梁速度迭代不恢复严格资格

- 431050c/872原SDF底梁6及原静态环境、无机器人/控制器、position16/TGS/CPU/external0，唯一requested velocity4→16；每组960步连续clock/4秒，源前后同/exit0。只研究被动接触，不是完整机器人严格落桌。
- 原table_support_ready及96共同门最长0/0。末秒normal支撑均值.552446→.369983N、raw角速均值.067163→.090888rad/s更差；线速均值虽降.570→.353mm/s也不能当联合通过，拒绝16/保持默认4。
- 原场景API请求回读4/16，实际有效求解岛计数未知；TGS>4 SDK有行为变化警告，不将请求当实时island计数。Pydantic嵌套对象导出未保留字段，metadata唯一差异及真实属性回读共同约束，首次分析KeyError保留工具记录，不作物理失败。
- sampled after_physics已有4支撑pair normal/friction都读取，最后时刻摩擦0；不能泛化完整外力平衡/原生getter错误。S01/S02/E03反例仍有效，当前仅否定提高这个requested计数的修复。raw完整保留round17_velocity_convergence。

## S07 — 第18轮外力更新选项未恢复联合门，数值设置路线停线

- 同431050c/872与17新4对照，唯一既有TGS enableExternalForcesEveryIteration属性off→on；body/controller config完全同，CPU/质量/重力/材料/SDF/原门保留。960连续原生样本、exit0/源前后同。
- 原联合96最长仍0，支撑力1.962N条件全程0。末秒Fz均值.428404N、角速均值.024613rad/s，最后Fz.024835N（不当作均值）。虽然raw角速均值较低，载荷资格没有改善，拒绝on/保留off。
- 只核已有scene attribute回读true，effective native effect未独立认证；法向/摩擦4已缓存support pairs都读，零摩擦末帧不当完整force balance。此处无机器人，说明该被动瓶颈不需要跨臂接触才能出现；不否定C10旧turn干涉、不能命名SDK getter错误。
- 两个数值设置候选无原联合改善后停止这条路线，广审碰撞表示与原支撑几何再选可证伪因素。raw/source/readback保留round18_external_force。

## S08 / E06 — 原始SDK碰撞回退与正常退出不能认证请求方案（第19轮）

- 19/431050c/872原板SDF→none资产完整作者属性/关系、全部点/面/尺度/材质与原源相同；原source SHA682610...、triangle variant2b3892...。final USD为none/kinematic，但原SDK报root中间dynamic导致none不支持、实际fallback convexHull。原联合960中最长948、末秒Fz2.425389N/omega.014866rad/s属于该回退场景，不能叫triangle物理验证；初版supports_candidate=true撤回，修正版false。原数据/错误保留。
- 原板55304顶点/114060三角，合并同位置后闭合但非凸：area1.289037 vs hull1.187250、volume.005612692 vs hull.005781250，凸包填孔，拒绝将该回退作为原几何方案。并非所有mesh的凸包等于原物体。
- 离线audit在finally app.close后exit0却没verification；主线程误据exit推进，20convex准备随后资产缺失、World前FileNotFound，source872前后同/没有物理样本。修复以数值指标/独立error/verification文件判断，不能靠退出码；错误承认并保留，不计抓持路线失败。
- 可执行最小修复用GeometryObject现有kinematic_anchor在构造时明确既有静态板角色，避免SingleRigidPrim包装先出现dynamic；仍保留原孔洞/triangle。20短检查先于probe验证rigid+kinematic且原报告无fallback/error，再判断原联合96。静态环境初始化不等于锁动态装配件，不能修改原门/睡眠/重力/碰撞或重置状态。当前尚待20结果。

## S09 — 第20轮构造期静态原三角面没有形成有效严格落桌证据

- 现有kinematic_anchor用于本来就静态的板，原非凸孔洞/点/面/尺度/材质保留，base6 SDF/质量/重力/velocity4/position16/TGS/CPU/externaloff与原门同。probe前root rigid+kinematic已核，无dynamic-none fallback，生产C872源前后同、原SDF资产不改。
- 26有效样本、27间隔后physical30/time.125000006519s invalid sample停止；SDK1065次PxShape material face index0xffffffff警告，completed=false但exit0。原联合最长0。failed同clock已知normal[-.188979,.250117,-1.547e-8]N/friction0；微小负Z触发signed valid=false，非大向下载荷或唯一根因证明。已缓存4support仅已知力账本，不独立认证native cooked或完整力平衡。
- 拒绝板候选/不长跑，保留source431右避让候选与cbfd首柱物理回退、默认原SDF/velocity4/externaloff/全部原门。20/20累计上限已到，不启动21；固定workflow保持，后续需新的轮次授权。
- 20被动无机器人/相机，不提供三腕影像或夹稳/自然释放/五件证据。failure_review/raw/source/asset与原material warning保留，不能凭exit0/少量valid rows当通过。

## V23 — 窄化判稳修复通过首插，首次释放发现腕目标分支跳变；当前暂停

- 最新用户指令“这轮结束后暂停，同步文件，下载最新视频”已执行；本批3/10轮完成（累计21–23），不部署、不运行第24轮，余下7轮处于暂停。v200 runner666134已退出，不操作其他项目进程；exit0只表示正常退出，failed=true/formal_success=false。
- 第23轮v200共25928步：原post_lift_restore_gravity已通过，证明V22宽泛2秒判定导致短阶段超时的问题已修复；首柱末段插入及原physical_hold_check96在25854通过，随后实际进入底梁开夹，在25927/phase_step1触发beam-exploration-seated_base_displaced。右高位避让/右侧抓持/左释放/第二件/最终五件均未到达。
- 开夹前base相对落桌锚点约0.000371mm/0.02399mrad，支撑2.238N；第一个开夹物理间隔后约0.100065mm/14.0491mrad，支撑31.040863N，触发探索10mrad角度门，尚未越原installed3mm/50mrad位姿门。右inner finger—base6已知目标接触24.282712N，另一指0N；base6—assembly_support法向Z31.040863N。不能仅因新门更严就称无物理扰动，也不能凭一间隔证明长期失稳或直接放宽门。
- 释放控制另有明确异常：右腕3实际约−3.44163rad时，SDK发布目标变为+2.84156rad，相差2π；原policy三个归一化函数用本轮实际关节输入可离线复现，目标误差仅7.15e−7rad。机器人preserve_continuous_arm_joints=true，但policy先把reference也wrap，释放loaded路径没有走保留native分支的idle anchor。与此同时夹爪目标q从0.664287直接到0，实际q到0.657557；两因素同间隔，未分离因果贡献。恢复后先以保留native关节分支为一个主要物理因素，开夹命令和保护保持，再验实际目标/运动/空间力/三视图；本次只诊断、未修此源代码。
- 5条invalid hold均为同一physics25931的base6/right contact_missing：1条在命令释放、4条在终止重复；此前无失稳记录。不能把命令开夹丢双指本身当异常，也不能把5条统计成5次独立失稳。原生四边界stride8末记录cycle25920，释放末态使用同physics25931的contact snapshot与SDK发布记录，不能虚构最终native target tensor回读。
- 本轮source873项前后逐SHA不变，manifest7ae2be213f05471ee98d6565550d1b2ee187d29748af6a91b20522154e21fd14，5启动工具字节同冻结归档。至25853共105914同key接触记录与v194对象/TCP位姿、夹爪q和双指力逐数值差0；只证明该前缀所查字段，不推及释放后。L873零差；W实际运行源码和7修改文件同C，另3既存旧协调文档/8AppleDouble元数据差异保留在local_full_manifest_verification.json，不宣称W全873一致。
- 原严格落桌stable_count最大仍0；10950按位姿双窗+支撑联合96放行，支撑2.300365N。正式严格装配未通过。原40N/3mm/5deg/双侧96/最终共同240、阶段timeout、重力、材料、目标/IK保持，原SDF板与TGS external off保持。
- 视频壳体修复保留：原collision实例/schema/材质/变换不动，只新增无碰撞视觉mesh。首段/中段、45–85度和末段三视图所查壳体可见；18原collider离线一致，原生前缀执行一致。完整三路最新视频已在D/round23_scoped_pose/episode_0000_live_videos/，总28313598bytes，每路3241帧/约108.03秒，服务器完整解码0error，两端SHA逐一相同；video_download_manifest.json与video_decode_validation.json可复核。终止额外frame25928实际physics25931，按物理戳对齐，不能套普通frame标签规则。
- 运行/预检原报告均保留：3241实体样本每次44poses，所查跨臂/持件他臂重叠事件0；runtime/stage error为空不等于全覆盖。粗监视缺fabrica_fixture/fixture_support几何、通用stage两侧各9夹爪solver frame缺失且未来夹爪冻结；实体层另外覆盖48authored colliders。背景Scene根/45默认offset/native cooked/SDF/CCD/自碰撞/全力平衡未认证。75–85度实际28样本cross/他臂持件min98.917mm，旧指—腕min146.805mm。额外part2下降221/闭合81/携件2154采样仅名义预筛，不是后段原生证明，不复制柱3偏移或力度到2。
- 完整后审包completed_review.tar.gz为107245596bytes/SHA14c26887c87c390130065cef65ffa2b55ee0a41c6bdc91e870dcdff3d3d25875；释放窗口包release_window_review.tar.gz为6716168bytes/SHA1a49cd8a8000603554cc850f6934192e06838ec93310e155b175b212e31c72e5。补充release_command_branch_audit.json、postreview_conclusions.json、所有原日志/回退保留。原manifest/启动脚本、三视频和原始报告均已下载。
- round24_preparation的6项准备改动（最终退出一致性、总episode预算覆盖原timeout之和、既有cached净力读取扩大至后段）保留在独立prepared_sources/test_repository；511Beam/252相关通过、2skip，873测试副本+6patch核验0差异。全部未部署、未原生运行；新发现腕目标2π问题尚未修。若用户恢复，去掉worker65000整体截断，但先读此次失败链和现有源码，不直接启动旧准备包。
- 四协调文件同步W及服务器R/C/旧基线，ledger和准备包/证据状态同步SD，均以resume_20261014/documentation_sync.json与final_pause_sync.json的当次SHA验证为准；禁止自动公开或更新PR。准备源不覆盖C/L/W，运行回退不删除。

## V24-prep — 用户要求失败后继续观察5秒（无新增原生轮）

- 用户2026-10-14直接要求失败后多录几秒；选择默认5秒真实物理时间，duration可配置，三视图写同物理戳，首次失败时刻/原因/指标保持；成功/非Beam不追加。停止新动作，保留最后控制目标，所以它是该停发动作方式下的演化，不是自由落体或后续policy能力验证。
- 独立准备generate_demos尾录接入、beam_failure_video helper和15专项，整套准备由6增至9文件；不改C/L/W的873运行基线。检查覆盖持续真实步进/首失败结果不变/不调用task qualification getter/各镜头足额帧/无效时钟/缺native/场景移除/物理异常/失败后帧明确标签/所有selected构件仅用既有views。最新服务器检查与SHA见round24_preparation/post_failure_video_preparation/source_and_test_verification.json。
- 原生未运行、现有v200不能补录，暂停仍为3/10。下一次用户恢复先修已证实的右腕控制目标2π分支问题，再部署合并准备项，并实际验收三路尾段各约150帧、连续5秒物理时钟及失败前缀回归；不通过延长观察来放宽原成功或保护门。


## V24 — 首次开夹修复通过释放，自动避让分支及旧缓存接管仍异常；已暂停

- 最新单轮授权已执行：v201/累计24/本批4，唯一一次原生运行26520步、exit0但failed=true；原runner719229已退出，pause=true，不启动25。原五件装配及严格落桌未通过，formal_success=false。本批4/10完成，余6轮暂停；禁止自动公开/更新PR。
- 保留11源文件合并修复：明确释放分支使用native角度、5秒真实失败尾录、原最终退出/总体budget475996/既有cached净力范围准备。533Beam+252相关通过、2skip；最后3项未来阶段行为尚未原生到达，不能据离线检查称完成。
- 源码876项manifest SHA2cbd88fac38a1ec6d03ade2933f73383f26edb5d1ec4fdc29e2316edf482877a，原生前后逐项相同；L876零差，W运行源码相同、3既存旧协调md/8AppleDouble元数据差异保留。W HEAD431050c有意未提交。原v200完整873回退SHA d3228374c1744a0c67f6ed1aea917b7f853db138699954e8d2fe65e2285e297b。
- 首插及原physical_hold_check96于25854再次通过。至25853共105914同key对象/TCP位姿、夹爪q/双指力与v194数值差0；前2500原生1252四边界q/target/可直接比较body位置差0。原严格stable_count最大0；10950仍仅位姿+支撑联合96、2.300365N探索放行。
- 首个开夹25927已修：cycle25928/physics25931生成、SDK与native position targets逐分量差0，wrist3保持−3.4416294098rad，实际首间隔变化1.38283e−5rad。底梁释放关卡完整通过26478。不能据此称整个释放控制问题彻底修好。
- 完整后审否定“所有释放路径已覆盖”：新helper有25939–26036共98tick缺口，进入既有idle-clearance分支。25944原生q wrist3−3.39515305、实际joint target+2.84051991，差6.23567295rad；同SDK记录中IK正确native分支−3.43922397被arm_joint_override跳过。该路的显式joint归一化仍有问题，见release_control_handoff_audit.json/release_control_native.jsonl。
- 随后hold_3_approach_clearance退避第41步/26519，首柱3触发原3mm位置滑移保护，实测4.18364755mm、双指27.950/11.481N；不是仅新位姿门引入的假失败。26480实际发布旧局部技能缓存目标，最大实测差1.04851508rad，目标对25926旧缓存差5.45108e−8rad。源码_hold_joint_action用旧_command_reference_q，未走实测限幅；IK失败/分支守卫具体触发未启用日志，不能断言唯一原因。
- 空间证据：26520左forearm与右left_outer_knuckle包络重叠.538mm、left_outer_finger重叠2.976mm，24张退避/失败/尾段三视图已查看并与原生q/目标、44实体姿态、物理戳对齐。普通原生末cycle26512，恰在首次失稳的cross-arm pair原生冲量未取得；此前所查18夹爪normal净差0也不替代末间隔力平衡。右高位路径/右侧抓/左释放/后续件/最终共同240均未到达。
- 失败后真实继续1200步/5.00000026077秒，三路各新增150帧；camera ReferenceTime均30fps连续、physics每帧8ticks；151native samples含失败首帧有效、108cached pairs全valid。失败phase无unfiltered net sensors，不能认证完整外力账。保留最后发布目标且不发新policy、不重新判定；尾段base角漂移峰137.669mrad、结束8.622mrad，不能把末帧回稳改写成原成功。
- 壳体可见修复保留：早段4/8s、中段、45–85度、退避/失败/尾段所查三视图可见壳体正常；遮挡范围不算缺失，未逐帧认证全片。原18机器人collider/实例/schema/物理属性离线不变；105914零差前缀支持所查范围物理未改变。
- 完整三原视频已下载D/round24_native_release/episode_0000_live_videos/，每路3465帧/115.5秒，总30690626bytes，服务器完整解码0error，逐视频两端大小/SHA同；video_download_manifest.json/video_decode_validation.json可核。主包124300980bytes/SHA fdcaccb322a2fa2156a03a4e95d5e808be80ec06928501ef46ba1e940427e818；失败窗口3597395bytes/SHA3e96c9c18234d3fcb7d2767775a248eee41a96d7a61cfd32a6e83b72303756a0；控制链2024804bytes/SHA15bb175b69c3475e0215c93bc3b8b415661e7471b3f06c6d3129f9ad54f90ebe，均已下载核验。重点视频1:48后的开夹、退避与失败尾段。
- 固定后审原报告：runtime3315/stride8、stage415/stride64/passive，错误为空但实体层有上述2个cross-arm候选；48colliders/44poses、当前原生43未知offset（旧离线45）、4桌腿无collider；粗层缺2个fixture几何、通用stage每侧9夹爪frame缺失/未来夹爪冻结；背景/cooked/SDF/CCD/self仍未证。75–85度28实际样本cross/他臂持件min98.9169mm、旧左指右wrist2min146.8051mm、无重叠，只限该窗口；名义3384扫掠不能认证新出现的自动避让和旧缓存回拉路径。
- 下一轮若获恢复，先独立审查demo_policy idle-clearance显式joint覆盖与plumbers_block_ur5e_skills缓存/失败保持链，统一native角度及实际已发布目标的控制接管，给所有回退分支保留原实测跟踪/步幅限制。先用本轮实际释放终点重建双臂/完整开夹/持件轨迹，再验首次开夹、98tick分支和退避第一指令；不先调夹力/放宽成功门。本轮不再修改运行源码或仿真。
- 本轮总结及可证伪结论在round24_native_release/postreview_conclusions.json。4协调文档、ledger、active_progress及最终审查清单以当次documentation_sync.json/final_pause_sync.json的SHA校验为准；原日志/回退/三视频保留。

## T02 — 早期冒烟与v201耗时差（只读证据分析）

- 本轮仅分析原始记录与代码；未改运行源码、未启动25。W HEAD431050c实测一致；本机v201 manifest876项复核结果见`outputs/beam_grasp_repairs_20261012/runtime_cost_review_20261014/timing_evidence.json`。当时服务器SSH握手被重置；后续T03已重连、逐项核验源码并补齐同步，最新证书以T03为准。
- v201 started15:22:58/finished17:20:00，117分02秒；主循环26520步/末平均4.111步每秒约107.52分钟，约110.5秒物理时间；余约9.5分钟合含初始化、失败后1200步/5秒物理尾录、收尾等，未逐项计时。主循环非大部分时间卡在初始化。
- 最早成功冒烟是plumbers_block右臂base_prepare，16676步；不是Beam五件严格物理装配。早期Beam support3931步即set_down失败，末3900步平均16.872步每秒；当前底梁set_down边界10950、总26520步。不同版本/任务覆盖，不能把约4.1倍吞吐差当受控因果消融。首柱pickup_clearance2607步、assembly_clearance4392步，进度日志估算合约38.5分钟，含搬运/转向及阶段控制。
- 已证步数增加与吞吐下降同时存在；两代均240Hz、相机分辨率相同、原冒烟已有三视图stride8，当前render interval5→7，不支持单纯因新增三相机/提高物理频率而变慢。当前四边界/每8步原生采样、5构件/双臂/接触、同步JSON、实体/阶段检查及局部IK均在执行链；各项耗时占比未测，不宣称诊断或物理计算是唯一主因。原约束辅助与当前纯接触验收不同；底梁严格落桌仍未通过。
- 保留全部原日志/回退，暂停不变。下一步建议先提出分项计时方案，经用户确认方法后再实施；先量policy/IK、物理步进、接触采样、几何检查、渲染/写盘各自开销，再决定优化，不能放宽成功门或直接停掉已确认检查。

## T03 — 性能实验：重复状态/接触计算热点；诊断关闭无大幅加速

- 2026-10-14 GPU历史补核：既有181条19:16:40–19:47:57总利用率8%–28%（中位15%），覆盖A/B/B2；低总利用率时当前前缀仍约7步/秒。另有PhyAgentOS后台仿真约207% CPU，但记录无GPU逐进程利用率，未做独占，不能排除资源竞争。v199/200/201同3480步6.922/7.045/6.963、同17040步5.092/5.136/5.012，重复偏慢；这三长轮无同步GPU历史，终点不同不直接比末均速。范围及未证项见交接末节，无新实验。

- 2026-10-14只读补核：旧`outputs/fabrica_canonical_demos/beam`在2026-09-22保存完整旧流程21449步、96动作阶段后success=true；最后21420步均速17.439，主循环约20.5分钟，Kit整进程1560.129秒≈26分钟。两端结果/日志SHA已匹配；属旧固定/锁件辅助标准，不作当前纯接触物理成功。固定旧代码对照4b5a894的D2只跑3500步，末3480均速15.039；96阶段名称顺序同历史留档，但历史全部源码/环境身份未证。不能把短测15.039或当次历史26分钟互称同版本全程实测。
- 当前v201配方163阶段，总预算475996不是预计成功步数；第24轮26520任务步、长程平均4.111步/秒、主循环107.5分钟/整次117分钟，前46阶段通过，在第47保持退避阶段41步失稳。首柱插入及底梁释放通过，换手/余三件/最终240未执行，严格落桌未过；另1200失败观察物理步单列。前3500步6.900–7.033仅描述短前缀。无新仿真/生产修改，暂停保持；原件路径和详细口径见交接末节。

- 用户已授权性能比较及必要仿真；本次6次串行、有界运行各3500步已结束，不计装配第25轮、不恢复释放/退避修复。当前C生产源码876项最终逐SHA无差，W HEAD431050c不变；原第24轮物理未通过项和暂停继续有效，不公开/更新PR。
- 权威工具/原日志在SD/performance_experiments_20261014，完整下载证据在D/performance_experiments_20261014/evidence，中文总结为同目录上一级`性能实验结论.txt`。A/A2完整配置；首B诊断关闭被默认值覆盖，仅作CPU剖析；有效关闭组为B2。旧D首次构造尚无任务，初态覆盖不成立，D2在首个动作前取得实际41collider/37pose；欠项、原件和修正记录全部保留。未执行C录像内部历史方案。
- 关键单因素对照：同lift2640–3120共480步，完整83.209/86.231秒，B2有效关闭82.821秒；与复跑波动相当，不能归因数量级减速。B2实际env0/native boundary调用0/边界日志缺失已核，其他接触/实体/阶段检查和三视图保留。原B剖析后速度异常不作加速证据。进度反推时间有SPS四舍五入误差，不解释毫秒级差异。
- 已测Python热点：64步lift中136次状态getter、362次hold、426次contact metrics；deepcopy递归3062028次/1362外层，4.052秒（剖析窗15.270秒，含分析器开销）。直接调用方getter2.280/hold1.492/接触日志.264秒。inclusive有重叠，不相加、不外推整轮比例。重复控制/观测/录像读取及完整快照复制是优化候选，尚未生产修复。
- 原v201完整报告runtime246.7058秒+stage41.3276秒≈主循环4.46%；不能把关闭碰撞检查当主要优化。短窗GC约15秒量级，不支持已证内存泄漏或主要因果。长日志单臂持载约5.6步每秒、双臂持件段约3.0；阶段相关，双臂各函数占比尚未新原生剖析。
- 旧4b5a894的635代码/配置文件逐SHA保留、补充下载资产共享，非完整历史环境复原。实际背景同为warehouse；旧末3480步均速15.039、当前6.900–7.033，但任务阶段和物理条件不同。旧配方5处fixed_joint，首动作前5个Beam构件collider均disabled；不能泛化为全程都关闭，更不能当当前纯接触物理任务性能。旧D2预检41/37仅启用几何，disabled构件/4无collider桌腿及未来夹爪、背景/cooked/CCD/原生pair等缺测明确保留，不作安全或装配认证。当前48/44范围及原缺测沿用，路径未改。
- 现代组逐key2198条所查构件/TCP位姿、夹爪q、可用关节q/速度和双指力差0，源比较报告保留；只限前3499步和所查字段。全部6组截断3500、exit0，结果success=false/failed=false/phase_status=running，不统计为物理失败/成功。
- 18完整原视频已下载本机evidence各组episode_0000_live_videos，逐大小/SHA同，服务器全解码通过；现代每路437帧，旧每路435（现代对相机clock、旧对保留PNG数，旧绝对clock未证）。主包29574165bytes/SHA445504dae5e4027e818aa4703254a4024154ddf47f901ccb7bd432d77490182b；local_download_verification.json逐项可核。原日志/源码/回退不删除，server部分大归档已明确标记aborted.partial不可用，正确旧代码archive独立保留。
- 后续优先考虑同物理时钟只读采样复用及减少冗余深复制，保留判定逻辑/门/时钟和证据；这是建议，不自动改变常规方法或生产。若继续优化，以同seed位姿/实际与目标/双指力及关卡一致、重复计时改善超过波动为验收。完整五件、严格落桌和释放后控制缺陷状态不因本次性能实验改变。
- 四协调文件同步W及服务器R/C/旧基线，结果以本目录final_documentation_sync.json的SHA为准；该证书完成前标sync_pending，之后以证书为准。全部性能进程已退出，资源采样已停止；其他项目进程未操作。

## A01-prep — 用户授权的独立辅助路线（2026-10-15）

- 旧v201源码876项两端核验且完整回退已保存；旧C/W运行代码冻结，新A/A_L隔离。新方法由用户直接要求并授权，固定约束只适用于assisted路线，原纯接触标准及其未通过项不变。
- 静态审查否定直接复用旧符号附着/碰撞关闭/逐帧锁位姿路线：它们可把未接触抓取变成吸附、产生跳动或掩盖穿模。新代码只按严格双指接触后的当前姿态建固定持件关节，并按安装实测1.5mm/25mrad后建世界支撑；保留实体碰撞，不写零件位姿/速度。
- 新路逐件操作省去双臂换手；统一native动作出口覆盖旧2π显式joint和缓存回拉问题。78离线专项/相关检查通过；修复过metadata在任务加载后被覆盖的问题。尚无新原生结果，不视为修好/视觉通过。
- 首轮仅4000步底梁前缀，沿用原48collider/44pose前缀来源并重新记录实际；安装/释放/退避及全五件未来扫掠仍未验证。新路formal_success=false，辅助完整装配与视觉验收单独记账。

### A01 构造期反例

- 首次启动在随机化阶段终止：generic背景appearance group引用factory_background_visual，而新路已沿用physical完整16实体，没有该对象。实际物理0步，无机械臂动作、无视频，不计夹持停线或装配阶段失败。源码880项前后零差，原启动日志和回退保留。
- 修复：连同完整物理实体一起沿用对应随机化配置，新增真实seed0/layout505构造检查；同时辅助success要求self.success及目标判定共同通过，禁止仅五件临时接近目标就报流程成功。下一次为A02有界首抓验证。

### A02 首动作前控制入口反例

- 原生场景完成加载，48启用collider/44native body pose及完整可见网格/关节关系已记录；首policy动作被新guard拒绝，因为旧空闲compose路径仍含arm_ik_controller。没有发布任何机械臂动作，不计夹持未改善轮。
- 不能仅改local_skill参数就声称所有最终命令入口已统一。新路增加空闲臂native实际关节保持；机器人本来由显式joint覆盖的冗余IK项在出口删除，IK-only继续拒绝，并按8步记实测/请求/最终发布关节及同physics clock。相关行为增加专项检查；旧C未改。

### A03 辅助首抓/提起通过，搬运缓存边界拒绝

- 2965任务步（上限4000未跑满），1881双指真实native力接触且strict-pinch采样combined_surface_gap约0.01985mm后才建固定持件关节；2892原lift通过，2963 pickup_clearance通过。不是完整装配或全程视觉成功。
- 2964/assembly_clearance首步旧smooth缓存FK位置超前38.325mm、q超前.089167rad，scale=0，拒绝继续；不据此称物理夹持滑移，不放宽tracking门。另发现空闲臂每步实测保持会跟随重力慢漂，应以单次实测锚点保持并在active接管时清除。
- 880源码前后零差；54可见网格/37关节/48collider/44pose初态已保存，双侧关节图各15，初态native矩阵误差6.66e-16，不是非零q路径认证。warehouse仍为原5.1主资产。
- 失败后1200步/5秒三路各150帧已记录；native总valid=false由空闲左指两个既存reader未进入diagnostics缓存导致，部分body/关节/接触有效但不能认证完整尾段原生覆盖。下一轮复用已初始化全实体reader，不新建传感器或写静态位姿。

### A03空间后审与辅助v4控制方案

- 实测370个底梁—夹具完整三角网格采样无相交，最小0.232886mm；直关节插值名义lift有27交叠、最大.960mm，是被拒的候选路径，不能反写成真实运行穿模。笛卡尔直提保留。机器人完整32刚体/关节图同物理body匹配位置误差最大0.550738微米，无未匹配；仅覆盖A03所查姿态范围。
- 新辅助路去除纯摩擦preload/轨迹继承缓存；实际关节单次idle锚点、已夹稳命令固定、慢速直提、近目标后有界固定关节锚点对齐、全部6夹爪关节实际开夹后退避。旧C纯物理版本和验收门不改；辅助结果单独命名，不声称自然落座/承重。
- 辅助v4运行时禁止直接写Beam pose、清速度、切kinematic或禁用碰撞。对齐只改已到位1.5mm/25mrad内的辅助关节世界锚点，每physics≤0.1mm/0.2mrad，失败后不再更新锚点。
- A03原视频三路520帧已本机/服务器SHA核验和完整解码，关键夹持/提起图未见明显视觉问题。后续首柱、剩余件、全五件及全片视觉验收均未执行。

### A04辅助v4否定与v5有界累积

- A04抓取1881严格双指仍通过；5881触发lift4000步timeout、结果5882任务步失败。actual重新起步的微小servo不是足够的负载保持；原发布目标/实际有界但逐步偏转。完整735实体采样中465底梁—夹具可见mesh相交、max1.218414mm，否定v4路线，不能只因夹爪视觉贴合就通过。完整后审/视频保留。
- A04 2944原生样本均valid，尾录1200步/5秒、150/150原生有效；复用完整实体reader修复前轮空闲两手指缺测。源码880前后零差。
- 补正离线证据：先前1173名义路径lift仅80mm，实际recipe140mm；新1263姿态按完整140mm重算，夹具/底梁mesh零相交、min0.248384mm。旧报告不删，不能据前者认证原140mm实际扫掠。
- v5拟以阶段内有限Cartesian积累补偿受载偏差，距live TCP≤3mm/角度≤20mrad，仍统一native分支与步幅/跟踪门；去掉held路径的beam_coordinated标志，防止完成/settle绕回旧preload控制。231离线通过，不代替原生进展。

### A05停线、广审与实际姿态IK回放

- A05在lift第693步主动停止，2575任务步；首抓1881相同，实际提起没有改善，完整321采样中41夹具—底梁可见mesh相交，max0.070482mm。两个新servo候选均未改善后停线，没有继续调同一路参数。源码880前后零差；原生1288/1288、尾录150/150均有效。
- 控制链广审及SDK源码显示旧微步IK被CCD/branch recovery/backtracking反复缩小；normal world-Z0.2mm离线重放41个A05实测姿态，旧raw Lula CCD2/41，新局部FK阻尼逆解41/41，max46.485nm、q增量≤.000485rad、均.773ms，控制pose与同raw FK位置差0。只证明所查离线可解和计算量，不证明native新路线成功，不把重力/碰撞命名唯一SDK原因。
- 新辅助v6只替换held路径IK，不改接近/闭合门和旧C；所有指令仍经native分支/.02rad步幅/.12rad跟踪保护。原v4/v5拒绝路线完整回退保留。v6运行前必须重新扫掠并验实际运动。
- 54可见mesh首柱名义1614姿态可达，旧底梁assembled目标与board有20.808微米可见三角交叠。辅助装配目标共同+0.1mm以留正视觉间隙，所有相对配合不变；旧纯物理目标/正式成功门不改。名义upper指节.74rad曾在开夹起点与目标部件交叠~.87mm，是真实抓取值未取得前的候选，不可宣称未来实际开夹安全；应以新实测闭合q和held anchor重核。

### 辅助视觉资格与原物理夹稳分离

- 全54可见CAD检查取消目标手指视觉豁免后，A03真实指—底梁584对采样中292表面相交；FCL最大三角contact depth .810mm不等于已证体积穿深。旧force grip因此不能当作视觉无交叠证据。原native力/代理接触结论保留，适用范围分别陈述。
- v7辅助close以完整native q图预测CAD不相交的80微米名义间隙，再要求实际两侧5–250微米、无相交、12独立physics clock稳定。force仅观察，辅助固定持件资格明确叫dual_visible_CAD_surfaces，formal_success=false；旧C不变。五件名义校准都约80微米，实际仅底梁用既有实测锚点，其余native未验证。

### A06近夹实现正常，driver命令过慢；v8缓存目标

- 3207步close预算超时，882源前后零差；真实visible pinch704对采样无交叠，但min gap19.260mm，未形成新辅助抓取/提起。actual+.001rad被原PD响应约.166/tick折减，实际约.04rad/s；首失败保留，后写stop_request未生效。
- A06无接触原生响应+真实CAD闭环回放，旧2000tick未达，v8有限缓存目标707tick达12-clock资格/min81.929微米；实际接触动力学待测。新命令q领先≤.01rad/每步≤.001rad，随CAD gap缩小，≤40微米不再往内追。
- 两个未来风险合并修正：final assembled旧split轴宽限会代替严格norm门，删除；world support锚点对齐延迟至实际全开夹，避免闭夹内移动零件。均不改旧C。
- 帽1缺preshape的模型在q.167–.349报夹具交叠，生产在高处先q=.55；补正6189名义姿态无该冲突。原错误模型及日志保留，不称实际穿模。装配界面29.8–44.7nm表面重合仍原样报告，不是忽略持件。

### 辅助空夹回收的自身碰撞（A07前）

- part2旧左park在距目标27.886mm时可见指尖撞上臂，原18mm完成门前就不安全；同方向缩短20%并重算后续取件。帽0部分风险发生在原门以后的精确端点，区别记录。
- 完整CAD自碰撞须保留所有非相邻臂/夹爪实体。直接机械关节对及原夹爪内置housing/inner-knuckle轴承界面精确列出，不能整体跳过同机器人。
- 原A06三视频每路550帧已全解码/SHA同，18关键原帧已看；只是宽开夹靠近，不是抓取成功。新增CAD监视400实测entity采样无>1um相交，同步记录未导出仓库/采样间CCD/cooked形状缺测。

### A07实测新CAD夹持与局部逆解直提有效，阶段衔接零步长错误

- 双侧可见表面近夹2004步达标，实际直提约115mm于2594过原lift门，324完整entity采样含非相邻自碰无>1um交叠；native1300与失败尾150均有效。不是仅离线通过。
- pickup_clearance的object servo先调用orientation-only(max_position_step=0)，位置积累positive-step断言抛ValueError，尚未发布该阶段新动作。v10只在正位移步长启用位置积累；完整transport调用回归覆盖该衔接。物理和几何不变，保留v9与首失败视频。

### A08b搬运通过，放置末段指面CAD相交；腕部光心遮挡

- 7177步，2004新CAD夹持、2594直提、2683取件清障、6086装配清障、6503悬停。7176右内指/base三角相交参数125.896微米即停，非目标CAD0。3592原生+尾150全部有效，源882/工具7前后无差。未安装/释放，不是五件成功。
- 异常伴part姿态约.0086rad/高速瞬变，cached指pair为0，缺非目标完整力；SDF支撑/约束漂移只是待证解释。新路线准备在目标上3mm测量冻结，完全开夹后由world support缓落，防止持件承受末端压紧接触，不能把此改成纯物理成功。
- frame460/479右腕连续黑而front/left正常，646恢复；native挂载变换得到光心位于forearm包围/开放mesh绕数.58–.86区域。相机眼移前侧，须扫掠和实测新视频，不静默丢黑帧。
- 增大nominal指面余量至230微米但保留5–250微米资格；实际A08b锚点离线回放暴露旧左右label固定居中方向错误，新用真实CAD间隙有限差分梯度，原发散日志保留。尚无v11原生结果。

### v11实际锚点居中与3mm安装缓落的离线结果

- A08b实际闭夹锚点以真实CAD间隙梯度居中一次71.377微米，双侧230微米；后四件230–230.14微米，无相交。保持5–250微米/12clock资格；不是原生成功。
- 旧固定左右label居中方向会使真实锚点越调越偏，有限差分负梯度纠正；失败候选日志保留。原80微米路线未触发此分支。
- 新base1289、后四件6144姿态覆盖开夹再world缓落，非目标CAD仅29.8–44.7nm装配面数值接触。相机右Y+16cm/左Y-16cm、Z8cm的镜像眼位，在8330实测/新名义姿态无光心AABB候选或自身臂中心视线阻挡；编译光轴dot1.0。需要下一轮真实视频。

### A09原帧否定静态相机预检；以native物理后回调驱动绝对USD相机

- v11新腕视角在真实frame46看向前臂，961步人工证据停止，未测试新夹持/安装。前述SDK Camera默认X轴不适用于仓库SingleXFormPrim相机封装，已撤回；单看模板四元数dot1不能认证真实视图。
- USD authored link世界姿态和native初始化关节状态不同步，初始差.617m/2.094rad不能当固定local-frame修正；mesh局部坐标来自同次prim/owner变换相消，不因此改动。
- 未运行v12的world.render包装被独立审查否定：正常World.step渲染dt非零直接app.update；v13用SDK SimulationManager.register_callback(POST_PHYSICS_STEP,pre_step=False,order100)。只更新虚拟相机，依据唯一native腕/可见指垫中点，实际USD world矩阵读回，正常/尾录均覆盖；需要新的原生画面验证。
- 视频帧不能直接frame_index*8对应task：A08b frame250实际task2008、frame460实际3688、479实际3840，原video_frame_clock保留。camera_eye_audit_corrected_clock按真实记录重算；render reference与物理clock的独立原记录不改写。

### A10零任务步的相机初始化异常与observer等待

- 未执行新控制：missing camera prim异常在first policy之前；Kit退出0、原collect为running/steps0不代表通过。没有三视频，明确记录初始化缺测；源883/工具7前后同。
- 虚拟相机必须读取真实传感器后端注册路径，显式确认/创建Camera schema；不可仅按metadata猜stage路径或节点类型。记录请求路径/实际路径/类型，禁止对native物理body应用相机变换。
- CAD observer需要处理finished但entity trace不存在/空文件；v14以真实0步输出构成的离线文件夹具验exit1，防止等待永久挂起。并未启动额外原生实验。

### A10b辅助新近夹已实测；静态捕获目标漏掉布局随机化

- v14/source883/关键工具7运行前后SHA同，8504任务步证据停止，formal/assisted success均false。2073实际双侧CAD间隔244.963/235.728um，2663直提、2752取件清障、6155转向搬运、6572悬停通过；原物理force资格false/双指cached0N。新230um近夹及局部FK搬运有原生进步，不升级为安装或五件成功。
- 原assembled被layout505变换至[.7844409166,-.1205665433,1.0026]，新静态capture别名仍[.7,-.22,1.0056]，错84.441/99.433mmXY。此事实否定新增静态target名字自动跟随原随机化的假设；应在运行时原目标变换后施加world+3mm偏移，统一servo/完成门。当时按用户要求暂停；该映射后经B01修复、B02实测正确放置验证，不以名义7433姿态冒充这条错误实际place路径预检。
- 实际1063个完整54mesh采样含非相邻自身对无>1um相交，目标指面最小125.716um、非目标最小14.103um；失败尾段未在此entity流、采样间CCD/warehouse/cooked仍缺测。native4252+尾150均有效，不能由CAD零相交推断完整载荷或全场景证明。
- v14按实际每episode传感器路径修复初始化，1214相机矩阵记录误差0；20更新异常只在已完成尾录后。原帧显示近端手指遮挡夹持区，故“光轴dot1/无自身臂中心射线遮挡”不足以验收视角；后续需纳入完整夹爪遮挡。静态侧向候选仍有fixture粗包围候选，未采用，未恢复仿真。
- 四节点三视角短片和三完整原视频已下载、大小/SHA同、完整解码通过；原始失败/尾段/错误日志保留。最新用户指令要求A10b收尾后暂停，源码停在v14，恢复前不修改/运行下一轮。

### B01五轮新批次：全链审查提前拦下未来阶段错误（原生待验证）

- 用户已批准五次全面审查闭环及辅助专属交接；总入口仍为BEAM_REPAIR_HANDOFF，辅助细节为BEAM_ASSISTED_HANDOFF。历史B01/A11b启动准备；最终0动作工程失败闭环。最新B02后暂停，完成2/5，旧C876冻结。
- 除A10b捕获目标映射，还发现并修复最终object_at_target不受解释器支持、全开后未等缓落完成即退避、辅助终局复用纯物理table clock/锁件静态override。现用真实对象解释器及独立native12/240共同pose窗，不重新引入未验证的原生速度硬门。只复用现有view，不在资格getter初始化物理wrapper。
- 新完整夹爪多点first-hit预检用旧Y视角34/34失败作反例；Z=.12候选在稀疏268检查通过，但全量找到未来part1的32次fixture遮挡，拒绝；X负侧Z=.08全16992眼位通过（已保存8496姿态、54visible mesh），不能外推未导出/采样间/真实渲染。
- 配置/接口296项检查及4个实际随机布局通过；启动证据和实际配置签名绑定源码，原生首动作前原地渲染三视图再检查。v15b/source887/tools9及回退封存；这属于合并工程修复与预检进展，尚未新增装配阶段。

### B01首动作前真实图像拒绝与物理/渲染姿态分离

- A11b0任务步/0policy，native关节已初始化而USD机器人body仍authored；同stamp32body最大617.358mm/2.094398rad。render-only暖相机和正确相机矩阵都不能证明实体画面新鲜。
- 三原图门拒绝后完整收尾，source887/tools9前后同；无原流程视频，仅3原PNG。B02先量化native/USD，再验证SDK结果发布，禁止盲改视角掩盖实体姿态差；成功须实际图像与native证据同钟。

### B02未来阶段审查：最终入口与受载路点过冲

- 原最终helper通过不保证实际成功入口通过：辅助_check_success仍用旧速度，且terminal把phase_index置len；新资格证书须绑定本episode/phase/五件native view与clock，terminal只同钟重读，不重启240窗。330回归含真实入口链反例通过，原生最终尚未到达。
- 原末insert低至assembled+0.82mm，再回升3mm；截断目标后，gate_overdrive=True仍允许cached command越界。真实helper数值反例：actual3.2mm、previous/target3mm，返回2.8mm；False截3mm。只禁越过waypoint，保留目标/步幅/门，不能证明动态零件无越界或无穿模。
- B02新四件名义路径完整6134状态；连底梁7423全54mesh无>1um交叠，16972camera检查通过。仅名义/所导出实体范围；新SDK结果发布、实际图像及动态释放仍需A12原生验证。

### B02闭环：底梁辅助释放新增通过，首柱实际误差超出名义预检范围

- 当前B02/A12已结束并按最新用户要求暂停，B01/B02共2次闭环，B03–B05延期；v16b887源/9关键工具原生前后零差。完整五件和全程视觉未通过，formal/assisted success=false。
- SDK发布既有物理结果到USD解决617mm初始化画面错位，44body达到约3e-8m位置差，q/body/clock/计数/history未变；真实三图门通过，不增simulate、不写body状态。比单看相机矩阵更完整。
- 底梁真实目标映射修复已验证：辅助近夹、提起/搬运/正确放置、全开/解除持件、退避/park新增通过；7344全6关节近0、native12、115后续不同native时刻稳定。世界支撑辅助与纯物理承重分别陈述。
- 首柱辅助近夹/提起/搬运及insert00–06通过；insert07在task18272/physics18275因底梁6/柱3可见三角面相交参数3.317869um超过原1um数值阈值停止。实际XY误差1.043946mm，尚高于该路点8.666221mm。冻结pose只校正XY即恢复.814381mm间隙；仅姿态校正仍相交。横向误差是相关因素，唯一动力学原因未证明。
- 同clock该pair在柱3载荷约4.35719N，空间点和body姿态已对齐；11次IK收敛，所查local/SDK/native指令无旧目标跳变。名义7423状态/54mesh零相交不能认证实际/允许误差范围。后续恢复优先实际误差包络和近接口下降准入，不能只增epsilon/调力。
- 三原片各2434帧及五节点短片已下载SHA同、完整解码通过；首柱失败尾段5秒观察不等于安装成功，尾段仍保留目标且柱3下移约1.420mm。采样间/仓库/cooked/尾CAD缺测保留。证据B02_postreview_conclusions.json、三独立审查、B02_mesh_coverage_review.json及视频核验；当前协调状态以B02_final_pause_certificate.json为准。

### C01恢复：未对齐下降与物体空间指标W03.1

- 最新用户恢复辅助新批最多五次，装配要求右臂视觉扶持，旧物理冻结；每两次正式尝试复盘漏检，新瓶颈/跨模块/重复错误提前触发。
- A12第18264步按生产公式重算：Y纠偏−.091608mm，姿态补偿+.164470mm，已发布+.072862mm、Z−.981236mm吻合；.8mrad姿态差通过约224mm偏距影响位置，所查固定抓取关系微米级漂移。姿态跟踪差的具体动力学组成未唯一分离。
- 旧1mm进入/2mm退出放行约1mm偏差；理想预检未执行真实控制/缓存/最终SDK前限幅及误差范围。应统一绝对物体/TCP、检查最终发布持件路径、按CAD余量准入下降；不只改单一测量姿态开关或提高碰撞阈值。C01修改中，未原生验证。

- C01启动前纠偏：TCP Y+.072862mm包含必要的杆臂姿态补偿。按捕获SE3反算旧命令物体Y−.0918mm、XY.971→.872mm，不能称物体反向。11条回放中9条旧指令带约1mm偏差下降，是真正拒绝反例；见C01_controller_replay.json。绝对物体控制仍需几何/最终发布验证，动力学唯一根因未证。该条覆盖此前仅以TCP符号认定反向的过强解释。

### C批启动前未来干涉与检查复用

- 新右扶后旧part2搬运与右腕相交最高5.508mm、最终右park与顶梁相交5.337mm，名义全路径提前拒绝，未消耗native。part2双高位+60mm、删除最后park；修复6306状态中5960逐行复用+346新54mesh/self审查，min8.576mm。C01_nominal_bimanual.json/part2_clearance_probe/reuse_coverage_validation均保留。
- v1拼接末帧继承旧park被独立末帧检查拦下，v2 final精确继承retreat。仅v2 SHA d2ee32134586fc591a18817a2d9788a8ca8dd21cca0da6f8a53d9e169a890cb7允许进入启动门。最终误差包络/相机/真实动态仍待验证。W03.5记录检查器正反例及真实依赖缓存合同，不升格名义安全为动态成功。

### C01最终离线插入与自由悬停分支

- v17d生产控制三源不变，4410insert加1533实际free-hover补审，总5943案例/12315完整54mesh/self q状态，无>1um非目标相交。free-hover真实许可8mm/.15rad分离探针，不叫全轨迹误差上界；原失败74项只是工具将73hover误要求insert合同与总数差额，原报告SHA4341049b06bd3056635a372b2775bf628f2e24c0b68b54eefa25839025310564保留。
- 新final报告cc8c1d6cc0c8406eb633d72cebf9be9f9efb390ede0043cecec8d2b8af684bad；最终右眼[-.08,0,.12]8766全量过。actualcfg差异必须用真实配方payload重算派生fingerprint证明是camera-only，不能直接忽略整个指纹。原生尚未开始，五件/最终240/动态视觉待验证。


### C01/A13：派生浮点抓取身份清空跨步状态（W03.8）

- v17d/source889/tools18，实际12244步仅底梁抓取/提起/pickup clearance，assembly clearance主动证据停止；没有新增装配阶段。source_after889/18零差、1530 CAD采样无>1um交、5秒尾录/三原片保留；右扶和插入尚未到达。
- 原生6504/6505经真实三方法AST回放，derived relative position变化1.040834e-16m/quat7.632783e-17导致identity重置，注入的admission11及object_command清空；513实际发布角lead=.004rad支持没有积累。live私有state未直接读取，插入暖窗受影响为代码预见。C01_identity_native_inputs.json/replay.json与源码/工具SHA绑定。
- 旧预检覆写context、每case独立state、relative恒定，漏掉生命周期；5943瞬时几何case不构成跨tick正确性证明。C02只移除identity两个派生float项，稳定joint/attach/epoch/body/view/phase标识保持；真实多tick正反例和缓存lead最终q/full54mesh必须新增。不能仅增加静态理想路径采样或放宽门限。


### C02/A14：外部ACK非原子发布导致零动作退出（W03.9）

- A14 source889/tools18原生前后零差，physics3/task0。明确栈assisted_worker314 json.loads(ack.read_text())读空，root cat重定向先建空最终文件；完整ACK稍后写完不能恢复进程。trial异常JSONDecodeError，collect success/failed均false且phase仍running；按真实terminal_reason记启动中断，不误命名装配守卫失败。
- 379/4布局/16生命周期/600+8/3775网格及最终q推进离线通过仍不算控制原生验证，两轮均无新增装配阶段。无entity/motion/clock/支持trace/mp4/运行CAD，所有0为未执行，sequence=precheck_not_observed。
- 同类未触发风险：watch_visible_meshes44/62直接写STOP，worker366/367按存在即读。C03合并ACK/STOP共享原子publisher，整watcher AST限制到writer改动，运行889不改、critical19。正确发布前文件不可见、读到只可能完整JSON；真实读写并发正反例及第一动作receipt必须新增。
- 原始43文件/启动3图保留在P/A14_startup_abort_evidence，归档和报告两端SHA核验；无运动视频原因明确。流程下个常规复盘C04后，C03尚未启动。

## C03/A15：缓存修复原生有效，扶持交接清掉平衡偏置（W03.10）

v18a/source889/tools19，6545步。约20mrad角lead恢复，底梁转向/落位/释放及右非attached视觉扶持获取首次通过；首柱尚未抓起。6536双指235/235um，6544为136/329um，0穿透；最后右最终q切成6536实际q，肩部645.138urad跳变。源idle measured reanchor与同clock目标/姿态证据吻合，因果动态对照待C04。严禁据实际位置连续推目标/作用力连续；理想actual=target无法发现此类交接。v19仅support idle目标来源+纯stamp元数据，真实链旧645urad→新0，普通idle保留；没有放宽gap/改forces。见C04_support_handoff_control_validation.json、A15_support_handoff_native_inputs.json，原日志/旧源/失败报告保留。

C04/A16原生对照确认右扶active→idle保留发布目标有效：交接前两轮逐字段完全同，之后唯一已改源链保留target使6544 gap233/235um通过并持续到7016，旧同点因136/329um停止。排除底梁pose或driver变化解释该窗口，但不扩展为整个动力学/装配全部成功。same-clock原件见A16_support_handoff_native_comparison.json，终态完整源/日志SHA仍待收尾。

## C04/A16闭环：右扶已修复，持件约束相对变化触发独立模型门

v19/889/19，10641步、failed；原生右扶514样本gap233—235um，首柱抬起35.619mm，未插入。1330CAD/1735650pair/37动态无失败，2661SDK/发布同。10640的持件角模型残差1.945mrad>1.5门：腕FK同native仅.376urad，抓取关系变化约1.945mrad是主要误差，不命名为机械臂FK失准或一tick错位。A12全部696相关boundary行max1.174mrad，未漏已存超限反例；fixed joint实际localattrs/求解贡献未唯一定位。下一步先joint捕获读回/关系漂移分解及6DOF扰动生产链预验，不直接放宽门，不每tick清参考。报告C04_A16_control_failure_diagnosis.md/json、server decomposition SHA68d37d...；右扶对照及终态/媒体证书保留。当前用户要求C04闭环暂停，C05不启。SSH banner timeout导致最终新文档/证书同步pending，原视频本机大小/SHA已同。

2026-10-16恢复记录：用户批准W04.0流程精简并新D批最多10次；旧C04/A16和C05未执行历史保留。以冻结v19/889/19为回归，右扶已过、首柱pickup相对抓取变化未修；先流程同步后修复，逐阶段计时。

### D01 / v20：提起至清障的控制交接修复（尚未原生启动）

- 真实反例：A16第10637步首个pickup命令TCP跳变1.55991mm，最大关节增量0.005325rad；此前lift命令约0.2mm/0.000700rad。随后10640物件相对捕获关系角残差1.945mrad超原1.5mrad门。因果尚待D01验证，不把相关性称唯一原因。
- 改动：仅四个非底梁pickup阶段启用24原生tick平滑交接；继承上一最终发布q，首tick保持原命令，逐步恢复原步幅。捕获参考、1.5mrad门、严格插入/接触/右扶门不变。辅助joint只新增USD锚点读回日志，无物理参数改动。
- 源v20已封存889/工具20（新增共享流程工具），本轮完整检查406通过/2既有warning，185.377秒；针对性96通过/19.327秒，实际四配置35.190秒。完整检查只在冻结后执行一次；未用检查数量冒充原生成功。
- 新几何验证与旧计算复用证书正在检查，全部六门及文档同步完成后才能启动D01/A17。预检脚本自身负控匹配错误已保留日志，未启动模拟器，不计正式尝试；修正脚本不改冻结生产代码。
- 支持条件：首个pickup命令连续、原残差保护内通过旧10640瓶颈，后续阶段/实体/右扶合格。否定条件：同瓶颈仍超限，或新交接不能保持新鲜最终发布值；后续按实测分解重审辅助连接，禁止仅放宽残差门。

## D01/v20：负载模式切换的目标连续性（2026-10-16）

真实对照A16→A17：10448/10632全实测关节差0；pickup首适配器TCP跳变1.55991mm降至.211588mm，原始角步.5mrad；24tick继承/渐变后pickup完整通过，所存四边界角峰1.069244mrad，原1.5mrad未放宽。原v19/C04pickup10640的1.945mrad失败被跨过。下一未改assembly切换首适配器目标3.07829mm/4.171112mrad、q.008567rad，11747角残差1.960mrad失败，11748任务步；未插入。USD锚点创建读回误差5.924nm/2.02e-11rad；不是已证唯一动力学原因。

方法/否定边界：原生离散2938SDK与最终发布一致，首切换非stride SDK没有逐tick记录，适配器raw目标不能充当它。只给首入口补flag不足以认证下一阶段目标交接；新相邻边矩阵+冷下降准入反例及首8tick证据进入W04.1/D02。保留v20，不回退已有效pickup修复。1468 full54/self采样/652右扶及同钟绑定通过；三原片/6节点SHA已同；完整装配与全片视觉仍未通过。

D02/v21闭环：首柱assembly_clearance和transport_hover均完整通过，原11747超1.5mrad问题解决；insert00因3600tick超时，450样本native_count均0而实际下降96.723mm。命令轴向hold每tick重取当前位置导致持续跟踪误差可积累；没有固定世界高度/实际冷位移保护。四边界模型残差与SDK一致不代表实际保持；D03按W04.2验证持续偏置闭环，不放宽严格准入。源/工具889/20零差、3完整视频及6节点本机核验，原始D02_handoff_analysis和cold_admission_analysis保留。


W04.3 / D03启动前：当前v22源889已封存，SHA653ab091d35b99b97f50da9c16f7766622acdcaa9ff815341318357ebaeacf6a；关键工具20不变，v21及所有原件保留。本机889逐SHA同服务器封存清单。421整源检查/2既有warning，201.200s；四实际配置35.147s。7个实际关节逐tick演化的非线性FK偏差模型和7基线对照，A18真实起点三响应率oldfail/newpass；40插入阶段400偏移边界，共448个完整54/self样本及右扶均通过，65.767s。仅有限离线证据，原生和三视图仍待本轮验证。40严格插入新增固定冷世界轴参考、120um有界横向积分、500um实际冷高度越界即停；原150um/1mrad/12tick准入、100um/1.5mrad模型门、冻结抓取、全部碰撞及最终240门不变。

流程改进：新夹具先核运动有效性/速度/body/view及逐tick实际状态，再跑长几何；首次两次工具拒绝原件保留（22.256s/1.094s），分别为夹具漏运动输入和只读AST白名单漏len；修正工具后通过，运行源未改。SSH255后先核远端完成状态，本次suite/config均已完成，未重跑。媒体优化须绑定生产H264编码器：3x96帧原PNG的H264字节及统计一致，三路基线12.300s、新2.620s，仅小样本不外推全程；捕获/时钟不变。六门旧证据链只绑定一次，覆盖不减。计时配对按阶段+开始时刻处理手工跨PID，保留原事件且inclusive区间不相加。

最新停止边界优先：当前D03/A19完成修改、正式仿真、后审、三完整视频/节点下载与SHA同步后暂停，分析错误原因、流程和耗时。D04—D10不得启动。此前“v22待实现”已由本段封存和预检事实替换；不把预检通过当装配成功。

D03原生进展：A19同一次仿真首次达到12物理tick准入（14504已存样本横向78.378um），通过insert00及后续九段、release_and_lock、retreat/park；已观察到第二柱part2 move_above的17832步。当前仅首柱阶段完成，尚无五件/最终240成功；本轮结束后暂停，不启动D04。具体转换计数待不可变后审确认。


D03/v22机制结论：首柱冷世界参考+有界横向补偿已原生通过十段插入/安装释放；保留首柱成功段作回归。第二柱23272与右腕2相交.414831mm，前一有效清障模型残差仅2.191um/.302492mrad；碰撞后残差才放大，不能倒因果调力。名义同f平移/Slerp路径与实际生产路径不同，源/配置SHA一致不能建立走廊成员关系。具体反例/适用范围见D03_failed_carry_corridor.json和主交接；W04.4已收紧复用规则。仍无五件/240最终成功。
