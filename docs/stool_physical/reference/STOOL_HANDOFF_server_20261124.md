> 原文副本（2026-11-26 从 pcl-server 与 Mac 工作区取得，两处 SHA-256 相同）。文中链接和 `$R`、`$B`、`$S` 等路径指服务器或 Mac 工作区，在本仓库中不一定能点开。复现步骤以同目录上一级的 HANDOFF.md 为准。

# Stool circular / UR5e：辅助修复与纯物理路线

**维护约定**　修改本任务的代码、配置、验收口径、权威路径，或产生新的正式运行结果时，在同一次交付中更新本页，写明改动、已验证结论、遗留问题和证据入口。实时运行状态及授权以本任务服务器账本和最新人类指令为准。

2026-11-06 入口核对。新会话先读 [Beam / Stool 统一交接](../../PROJECT_HANDOFF.md)；下文保留本任务合同、S24 基线及 S25 回退证据。当前执行状态仍以服务器唯一账本为准。

先读根 [Fabrica 入口](../../PROJECT_HANDOFF.md)、[辅助技能](../../skills/fabrica-assisted-repair/SKILL.md)及本任务[合同](CONTRACT.md)。本任务不继承 Beam 的参数、结果或运行授权。

## 纯物理路线（2026-11-20 起，独立于下文辅助路线）

**结论（2026-11-24）：R05（最后一轮正式运行）核验成功，按“成功即停”结束，账本 `state=R05_verified_success_stopped`，5/5 轮已用。** 九件全部纯物理装入：`success-criteria-met`，53,580 步（约 223 s 仿真时间），墙钟 5,423 s，输出 101 MB；零超时、零恢复、运行时约束违规 0；双臂空手回位，最终无载静置 30 步通过。逐件误差/容差：帽 0 0.35/18 mm、腿 1 0.99/50.5、底板 0.16/19.2、腿 3 0.25/50.5、帽 4 0.68/18、腿 5 0.81/50.5、帽 6 1.80/18、腿 7 0.19/50.5、帽 8 0.50/18 mm，姿态均 ≤0.041/0.15 rad。另对“相对底板的原名义装配位”独立复核：九件均 ≤1.54 mm、≤0.031 rad。13 次抓取全部为 `pure_physical_grasp`（无固定关节、位姿写入或锁定，抓取时保持碰撞）。与同源码 smoke35 逐步相同（确定性复现）。审查见 `$S/outputs/stool_physical/rounds/R05_20261124_104131_P08/review.json`（含三路视频 SHA256）。2026-11-24 经用户同意下载三路视频及 review.json 到本机 [outputs/stool_physical_R05_20261124/](../../outputs/stool_physical_R05_20261124/)，SHA256 与 review 逐一核对一致。**交付需说明的改动：** 帽件成功目标在取件前按所插腿的实际位姿重定位（容差不变）；板上新增 108×108×3 mm 被动静态暂放台供帽件换抓；帽件两指严格接触以 PhysX 实测接触点配对并计入 1 mm SDF 余量；帽件“边夹—放直—深 10 mm 重抓”换抓流程；腿 1/帽 0/帽 8 限开度松爪。继续仿真需新的人类指令。

**2026-11-24 R05 后按用户“根治”指令去掉灰色暂放台（不计轮次的全程验证 smoke36–38，正式记录仍为 R05）。** 光学板本身只是一块 925×625×10 mm 平板（网格顶点只在 z=0 与 −10 mm），原 Plumbers 物理把它设为精确三角网格，顶面只在孔沿有顶点，SDF 帽件只在这些顶点处取接触，放下即斜 ~12°。改动：
- `physical_board_collision: boundingCube`：光学板碰撞改为长方体，顶面不变、孔被填平（Stool 不使用孔）；Plumbers 默认仍为三角网格。暂放台不再生成（`staging_pad` 改为可选并从配方删除），暂放格 XY 不变、目标低 3 mm。smoke36：帽 6 放下 0.00°，但板摩擦 0.5 时等待重抓期间自行滑 7 mm、斜 2.9°，夹爪偏 8.8 mm 超 7 mm 附着容差而超时。
- `physical_board_friction: {static: 0.8, dynamic: 0.6}`（与原暂放台相同）。smoke37：帽 6/4/0 换抓通过、8/9 件；帽 8 边夹斜 2.4°，`stage_place` 收敛按共享装配原点（帽件下约 100 mm）计为 5.2 mm > 3 mm 而超时，实际帽件中心仅约 1.9 mm。
- `target_object_reference_point_for_convergence`（仅四个 `stage_place`）：放置收敛也按帽件 CAD 中心判定，与已有的伺服参考点一致；插装及其他阶段不变。
- **smoke38 完整合同通过**：四个帽件放下 0.00–0.20°、全部换抓；九件 `success-criteria-met`，53,191 步，零超时/恢复，双臂回位，无载静置 30/30；验收误差最大 1.48 mm / 0.039 rad，相对底板名义位 ≤1.59 mm / 0.043 rad；墙钟 5,347 s，含调试 trace 170 MB。源码冻结为 `$S/outputs/stool_physical/packages/P09_board_root_fix/`（delta SHA256 前缀 `25e2c78c6d0ee736`）。账本 `state=R05_verified_success_stopped__root_fix_validated_smoke38`。此后场景不再有额外暂放台；交付说明中“新增暂放台”一项由“光学板碰撞改为同顶面长方体、摩擦 0.8/0.6”替代。若需计入正式轮次，需新的人类指令。

**授权与账本。** 2026-11-20 用户要求参照 Plumbers/Beam 在服务器修复 Stool 纯物理装配（精简、轻量输出、仿真快），选择：以 Plumbers C01 框架为底座、最多 5 次从头正式运行且成功即停、验收为“原合同 + 纯物理 + 双臂回位后无载稳定窗”。唯一账本 `pcl-server:/home/user/work/zhaoheng/stool_physical_20261120/outputs/stool_physical/ledger.json`。离线编译与短冒烟不计轮次。辅助路线账本、S24 基线和 Plumbers 生产树均未改动。

**工作树。** `S=/home/user/work/zhaoheng/stool_physical_20261120`，从 Plumbers 集成仓 `6f1de1d` 复制（`.deploy_parts`、`.runtime_python` 只读软链回 Plumbers 树）。部分克隆缺对象导致无法 `git commit`，改为每个候选包在 `$S/outputs/stool_physical/packages/P0x/` 存 `delta_from_6f1de1d.patch` 与源文件 SHA 清单。配方 `fabrica_stool_circular_ur5e_physical`；运行 `outputs/stool_physical/run_smoke.sh NAME TIMEOUT`（只开闭爪两份 trace），进度看 `outputs/stool_physical/watch.sh LOG`。每轮输出约 1–4 MB 加三路 10 fps 视频。

**代码改动（全部对 Plumbers 行为保持不变，Stool 通过配方开关）。**
- `fabrica_canonical.py`：连续物理分支从仅 `plumbers_block` 扩到 `stool_circular`（`_GENERIC_PHYSICAL_ASSEMBLIES`），抓取为 `pure_physical_grasp`，无锁定/位姿写入/碰撞或重力开关。
- `plumbers_physical.py`：零件 id 由对象名尾号取得；新增 `physical_close_force_max_seek`、`physical_close_force_max_command_lead`（默认仍为 0.03/0.001）；新增 `align_contact_boxes_to_grasps`（按选定抓取把严格接触盒的 +y 对准夹持方向，只改盒朝向和夹持宽度）。
- 任务 `_contact_box_pose` 支持可选零件系 `contact_box_orientation`，技能白名单同步；`plumbers_support.add_terminal_joint_home` 仅在存在支撑撤离引用时改写最后撤离；`demo_policy` 空闲臂保持允许 Stool 配方；`integration/phase_journal.py` 按任务实际零件名记录；闭爪 trace 每 120 步附一次只读接触详情。

**冒烟定位到的物理原因（smoke01–11，均为从头运行的前段）。**
1. 底板（75 mm 圆盘、近满开口）：Plumbers 的寻接触领先 0.001 rad 在 45° 斜夹时被约 0.0011 rad 静差卡死，指尖停在 75.3 mm；领先改 0.004、seek 0.08 后双侧接触。指垫刚硬时口袋固定的零件不能自对中（0/19 N），沿用 C01 右手的柔顺指垫 5000/15 后 5/10 N 双侧稳定，底板抓起、搬运、放置、释放、右臂回 home 均通过。
2. 臂位置驱动重力下垂与 kp 成反比：80000→0.42 mm、160000（S23 值）→0.22 mm，造成单指接触；kp 640000/kd 11313.7 后 0.07 mm。
3. 移动件按 143–145 mm 指垫中心加深会使指尖插入托盘口袋；保持 Fabrica 原 170 mm 夹点后指尖只超出约 7 mm，腿件双侧 4.5/4.7 N。
4. 六棱柱腿用矩形接触盒时两指分属盒 x/y 面，被严格接触拒绝；现按抓取对齐接触盒（腿 3/7/5/1）。smoke12 中腿 3、7 均完成抓取、搬运、插入（约 600 步）、释放和停靠，至 17.4k 步。
5. 帽件（18×18×30 mm）在托盘口袋里只露顶部：Fabrica 夹点下 PhysX 实测接触点位于两相对面顶部约 3 mm 带（4/8 N），但建模的指垫采样点在帽件上方 35–50 mm，被严格接触拒绝；加深 10 mm 则指尖撞托盘（零力停滞）。原生扫掠预检因 2 mm 纠偏余量与三角不等式运动界，在 14 mm 内一律拒绝，不适用。现对帽件 0/4/6/8 开 `physical_strict_contact_from_force_points_parts`：用实测 PhysX 接触点（力加权、有效且未截断）构造严格对向对，仍要求同轴相反两面、面距 ≤3.5 mm、另两轴在面内。离线用 smoke12 原样本验证接受真实双侧对，拒绝单指、同侧、偏 20 mm 和截断四种负例。smoke15（调试顺序 `@cap_first`，不计轮次）帽 6 抓起并搬到悬停位。
空臂回 home 由 0.24 提到 0.72 rad/s，前段约省 2000 步；旋转调速器前瞻由 0.010 提到 0.030 rad（运输有效转速约 0.13→0.22 rad/s，滑移门 3 mm/5° 不变）；当前约 9–10 步/秒。

**正式轮次。** 入口 `$S/outputs/stool_physical/start_round.sh P0x`（调用 `launch_round.py`：核账本额度/成功即停/上一轮已复核、磁盘≥20 GiB、内存与 GPU；冻结补丁和 SHA；不开调试 trace）。输出在 `$S/outputs/stool_physical/rounds/R0n_*`。R01 = 包 P03，2026-11-20 04:28 CST 启动，04:59 结束，**失败**：底板、腿 3、腿 7、帽 6（插在腿 3 上）均纯物理完成抓取、搬运、插入和释放后，在 19,549 步的 `assemble_03_part_4_close_and_attach` 以 `close_until_contact_timeout` 停止；零超时/恢复，1,882 s（96 ms/步），整轮输出 11 MB。腿 3 释放于 9.8k 步（smoke12 为 13.3k）。复核 `rounds/R01_*/review.json`，账本已记 `review_complete`。正式轮不开 trace，帽 4 原因用同序带 trace 的 smoke17 复现中；按 `@cap4_first` 调试序（smoke16）则在闭爪位姿门超时（指尖下降差 2.4 mm），与正式序不同，不作为结论。smoke17（正式序带 trace）确认帽 4 两指全闭（q=0.8）零接触：其 Fabrica 抓取中心只在顶下 4.8 mm（帽 6 为 9.4 mm 且只有顶 3 mm 带接触）。

各件 Fabrica 抓取中心距零件顶：帽 6 9.4、帽 4 4.8、帽 8 7.9、帽 0 12.0 mm，腿 1 原选峰端 3.2 mm。规则：浅的帽件沿近竖直接近方向加深到帽 6 已验证深度（指垫中心 = 0.1703 − (9.4 mm − 深度)），帽 4 0.1657、帽 8 0.1688；腿 1 设 0.1641 后抓取排序改选距顶 35.6 mm、宽 18 mm 的腿身抓取。正式轮从 R02 起开闭爪两份 trace（约 50–70 MB/轮）。仿真是确定性的：R02 在改动零件之前与 R01 逐步相同。

R02 = 包 P04，05:55–06:47 CST，**失败**：底板、腿 3/7/5/1、帽 6/4 共 8 件纯物理装入并释放（帽 4 在 21.8k 步释放，R01 问题已解），在 31,968 步帽 0 闭爪超时；3,087 s，输出 53 MB。帽 0 在口袋中倾斜 28.7°，抓取中心最深（12.0 mm）：q≈0.62–0.65 时帽上零力但臂已被推开 0.2–0.3 mm（先碰口袋），随后仅一指压在帽顶边（11.5/0 N），纠偏以 `force_imbalance_increased` 停止。R03 = 包 P05，帽 0 指垫中心 0.1729（回到 9.4 mm 深度；指垫中心上限由 0.17 放到 0.18，≤80 mm 平移检查不变），06:48–08:05 CST，**失败**：帽 0 已抓起搬运（R02 问题已解），但在 `assemble_06_part_0_insert_09` 超时（42,567 步，4,597 s，157 MB）。原因：腿 1 在底板里倾斜 4.18°。阶段记录显示所有件都在离座 4.4–5.0 mm 处释放（最后插入阶段位置 8 mm、轴向 50.5 mm 容差），松爪后自由落入，静置倾斜 0.8–3.6°，腿 1 在释放/撤离时倒到 4.2°，帽 0 沿名义竖直路径无法套上。

**座入后再释放（P06）。** 新开关 `physical_seat_before_release`（`plumbers_physical._seat_only_gate_settings`），复用 Plumbers 的 `_apply_seated_release_gate`，只收紧最后一个持件插入阶段（1.5 mm、轴向 1 mm、横向 0.75 mm、2°、24 步稳定），**不改验收容差**（与会改 success 的 `physical_insertion_release_tolerances` 区分）。smoke18 验证：腿 3 释放时离座 0.45 mm（原 5 mm），撤离后 0.69°。R04 = 包 P06，08:23–09:42 CST，**失败**：腿 3/7/5/1 均座入后释放，帽 6/4 也装好，在 43,619 步帽 0 `insert_09` 超时（4,704 s）。腿 1 倾斜降到 1.6–2.8°，但 95 mm 高的腿让峰顶偏离底板重定位后的名义位置约 2.7 mm，而座入门要求横向 0.75 mm；随后续件装入，已装件还会继续变歪（腿 1 停靠时腿 3 4.7°、帽 4 7.85°，仍在原 0.15 rad 内）。

**帽件跟随实际腿重定位（P07）。** `physical_rebase_targets_to_socket: {'6':'3','4':'7','0':'1','8':'5'}`（插座表取自本任务合同）：每个帽件抓取前新增 `*_rebase_to_socket` 阶段，复用原合同对底板用的 `rebase_from_object`，把该帽件装配侧目标（assembly_clearance、preinsert_hover、insert_00–08、assembled）按其腿的实测位姿重定位；任务侧新增 `keep_anchor_target`，腿自身的成功目标不动。**口径说明：帽件的成功目标因此相对其腿实测位姿，而非底板名义位姿；各件验收容差不变。**

smoke19（正式序）中重定位阶段通过，但帽 6 在 `insert_09` 停住（开始时与腿 3 对齐 0.02°/0.13 mm）；smoke20（调试序 `@leg3_cap6`，开运动 trace）中 TCP 按指令下降 15.1 mm，帽 6 座入（相对腿 0.23°、0.15 mm、z −0.02 mm）并释放。判断：P06 座入门的横向 0.75 mm/2° 以抓取前快照为基准，而插入力会推动腿；帽件在顶边夹持中还会绕圆峰扭转（R04 帽 0 相对腿 1 偏航 6.3°，圆峰直径 14.5 mm，不妨碍套入却永远过不了 2° 门）。P07 座入门改为位置 3 mm、轴向 1.5 mm、横向 2 mm、0.14 rad（仍小于验收 0.15 rad），只保留“不在高处松爪”的目的。由于仿真确定，先以与 R05 完全相同的配置跑全程 smoke21（10:57 CST 起），通过再用最后一轮正式运行。

**当前状态（2026-11-20 约 13:00 CST）：最后一轮 R05 暂缓。** smoke21 中底板、腿 3/7、帽 6（跟随腿 3 重定位后座入）均通过，但帽 4 在 `transport_hover` 持件时剧烈抖动（0.19 m/s、16.7 rad/s），过不了静止门而超时。帽件只能夹住露出托盘口袋的顶部 3–5 mm 带，左手指垫参数处在两难窗口：刚性指垫（Plumbers 左手原配置）抓不住帽 4（smoke22 `held_object_not_grasped`）；柔顺 5000 + 阻尼 60 则腿 7 搬运中脱手（smoke23）；5000/15 可抓但可能抖（R04 帽 4 通过，smoke21 帽 4 抖）。仿真确定，同配置跑 R05 会复现 smoke21，故未启动。工作树已回到最佳已知配置，冻结在 `$S/outputs/stool_physical/packages/P07_best_known/`。账本 `state=R04_reviewed_R05_held_pending_cap_grasp_decision`，已用 4/5。

**下一步候选（待人类决定）：** (a) 帽件“先边夹取出、放到台面、再从侧面深夹”的换抓流程，物理上最稳，需新增阶段；(b) 继续以调试运行细调左手指垫刚度/阻尼与夹持力带；(c) 仅对帽件夹持/搬运段接残差 RL。任一方案都先用不计轮次的全程调试运行验证通过，再用 R05。

**2026-11-24 人类选择“帽件换抓”，按此推进（账本 `human_instruction_history`、`state=R05_held_developing_cap_regrasp`）。**
- `physical_cap_regrasp`（`plumbers_physical._add_cap_regrasps`/`cap_regrasp_targets`）：帽件 6/4/0/8 从托盘边夹取出后，插入 `stage_hover → stage_place → stage_release → stage_retreat → regrasp_move_above/preshape/descend/close_and_attach/lift` 九个阶段，再接原 `assembly_clearance`。暂放位为装配原点 +(−0.14, +0.13) m，即台面 (0.56, −0.02)，帽件按装配姿态直立、底面离台 1.5 mm（放置位置容差 3 mm）；换抓沿原 Fabrica 抓取接近方向加深 10 mm（抓取相对位姿与附着目标同步平移，编译时核对两者互逆）。暂放目标为世界固定，不参与腿重定位。smoke24：帽 4 托盘接触带为顶 2.8 mm，换抓后为 12.6 mm（z 2.4–15 mm），双侧 4.8/4.4 N；`transport_hover` 约 180 步通过（smoke21 曾抖动超时）。
- 新问题：帽 4 在 `insert_09` 与腿 7 对齐（相对 0.01°/0.13 mm）却不下行；smoke25 运动 trace 显示下降 5.4 mm 后指令停在实测位置。原因：插入伺服在横向误差 > 1 mm 时进入横向对准并冻结轴向深度，而误差按零件原点计算，所有凳子零件原点都是装配原点（帽件下方约 100 mm），帽件在峰上零点几度的倾斜就读成 >1 mm。
- `physical_servo_reference_point_parts: ['6','4','0','8']`：技能新增可选 `target_object_reference_point`（`_target_object_servo_pose`），帽件持件伺服阶段改在各自 CAD 包围盒中心计算误差；完成门与验收不变。
- smoke26 在换抓提起时 `loaded_force_recovery_timeout`（11.9/5.4 N）：暂放的帽 4 在台面上倾斜 11–12°。光学板是带孔三角网格碰撞（孔半径约 3.3 mm，板局部坐标 12.5 + 25n mm），帽 4 中心落在离孔心约 (4.5, 9.5) mm 处，一条边悬在孔上而下沉。`board_hole_free_cell: {origin_xy_m: [0.47, -0.14], pitch_m: 0.025}`：各帽件按自身足迹中心吸附到板格中心（板局部 25n mm），18 mm 足迹四角距孔边 ≥1.6 mm。smoke27 中帽 4 仍倾斜 11.65°：实测孔半径仅 2.5 mm、格中心足迹内无任何板顶点，孔不是原因。帽件为 SDF 碰撞、板为三角网格，SDF 对网格按网格顶点采样接触，而板平顶只在孔沿有顶点，18 mm 帽件只能靠少数孔沿顶点支撑而斜靠（75 mm 底板覆盖大量孔沿，所以平稳）。帽件网格本身底面为完整平面、质量 10 g。
- `staging_pad: {height_m: 0.003, margin_m: 0.02}`（`plumbers_physical.cap_staging_pad`）：在四个暂放格下加一块可见静态长方体（108×108×3 mm，中心 (0.57, −0.015)），用基本体碰撞提供平整支撑；暂放目标抬高 3 mm。它是被动固定物，不约束任何零件，属于对场景的新增，交付时说明。smoke28：暂放倾斜降到 3.5°（原因未完全查清），换抓 5.1/4.5 N、接触带 z 2.4–15 mm；帽 4 经过渡悬停后在 `insert_09` 360 步内座入腿 7 并释放（相对腿 0.16°、0.08 mm、z −0.09 mm）。随后以与 R05 相同配置跑全程 smoke29：底板、腿 3/7、帽 6（经暂放换抓，20.5k 步释放）通过，帽 4 在托盘边夹搬往暂放台途中脱手（24.1k 步 `held_object_not_grasped`，掉落时 0.19 m/s、16 rad/s 抖动），说明边夹仍要扛住这段搬运。
- 改动：帽件持件力带用 Plumbers C01 零件 0 的 4/5 N（`physical_move_hold_force_control`，原 6/8 N 会把 3 mm 边夹挤到抖），暂放搬运与放置两阶段先试 Plumbers 默认慢转速（0.10 rad/s、前瞻 0.01 rad），但托盘到直立约需转 1.6 rad，有效转速仅约 0.05 rad/s，帽 6 `stage_hover` 跑到 2,800 步仍未完成、将超 4,800 步时限（smoke30，已停）。改为 0.25 rad/s、1.0 rad/s²、前瞻 0.02 rad（有效约 0.13 rad/s），并按物理编译同一制动公式重算这两阶段时限（7,060 步）。smoke31：帽 6 全流程通过（21.0k 释放），帽 4 已平稳搬到暂放台，但在 `stage_place` 触台时被 `held_object_not_grasped` 中止：持件 trace 显示同一步两指 5.8/5.4 N、滑移 0.17 mm/0.3°，`force_maintenance` 判有效而 `state_sync` 判 `contact_missing`，属严格接触的假阴性。
- 改动：实测接触点配对改为接受“明细缓冲已满”（截断只会丢点，保留点仍须通过同面、异侧、面距、面内检查；离线回归仍拒绝单指、同侧、偏 20 mm）；帽件按 Plumbers C01 销钉的做法开 `physical_contact_recovery`（≤24 步、两指面距 ≤0.5 mm，只保持、不认定抓取）。smoke32（全程）与 smoke31 逐值相同地失败（确定性复现），接触恢复从未触发，说明不是明细截断所致。smoke33 加只读的持件失败转储（`STOOL_HOLD_FAIL_TRACE`，只写文件），查 `state_sync` 判失败时的接触明细。结果：两指各有 31/36 个实测接触点、4–6 N，但左指力加权点 z=15.04 mm，超出帽件半高 15 mm 仅 0.04 mm，面内检查余量为 0 而被拒。Fabrica 零件是 SDF 碰撞、余量 1 mm（`sdf_margin_001`），实测接触点可落在网格外至多 1 mm。改为实测点配对在面内检查上加 1 mm SDF 余量（`strict_force_point_surface_margin`，默认 0.001）；离线用 smoke33 原样本：余量 0 拒、1 mm 接受；单指、同侧、左指上移 3 mm 仍拒。smoke34（全程）：底板、腿 3/7/5/1、帽 6/4 全部装入并释放（腿 1 于 36.9k 步座入）。但腿 1 松爪时夹爪全开到 85 mm，左腕录像显示上指扫到相邻的腿 3，底板随之倾斜 1.3°；随后 60 mm 竖直撤离把腿 3 拖倒到 27°，帽 6 掉落（96.7°），最终在腿 1 的 `unloaded_settle` 超时（38.2k 步 `timeout-failure`）。
- 改动：`physical_limited_release`（`plumbers_physical._add_limited_releases`），用于最后三件（腿 1、帽 0、帽 8），它们松爪时旁边已有就位零件。做法同 Plumbers 销钉松爪：`ur5e_release_supported_part` 只开到开度 0.34（约 33 mm，18 mm 夹持每侧留约 7 mm），两指 0 力连续 6 步才判定松开；撤离全程保持 0.34，竖直上升 100 mm 后才在 park 中全开。前六件的松爪不变。Plumbers 单测 45/45 通过。smoke35（全程，与 R05 同配置加只读持件失败转储）**通过完整合同**：腿 1 松爪与撤离后腿 3 0.1°、帽 6 0.3°、腿 1 1.2°；九件全部装入，`success-criteria-met`，53,580 步，零超时零恢复，双臂回位（53.4k）及最终无载静置通过；墙钟 5,456 s，含调试 trace 共 209 MB。
- **2026-11-24 10:41 CST 启动最后一轮正式 R05**（包 P08＝smoke35 源码，delta SHA256 前缀 `e8937ecd6f7ceae7`，`rounds/R05_20261124_104131_P08`）。结果：成功，与 smoke35 逐步相同，详见本节开头结论。

累计：调试运行 smoke00–26（不计轮次）；正式 4 轮，最好结果为 8/9 件纯物理装入（R02、R03），全部输出 804 MB，单轮 11–157 MB。

## 唯一当前状态

授权、轮次、暂停、执行身份和下一动作只由 [execution_ledger.json](execution_ledger.json)维护。服务器原件为 `/home/user/beam_integration_20261010_upstream6f57dae/stool_assisted_20261021/execution_ledger.json`。继续前核对最新人类指令和该账本；暂停后须有明确恢复指令，文档整理不授权仿真。

## 最近已封存的进展

S22 安装底件2及左件3/7/6/4，原 criteria 为5/9。右臂移到前视画面左侧后，已跨过 S21 的双臂干涉点；首失败改为 part5 闭爪超时。实测稳态对中偏差约0.597mm，超过原0.5mm对中领先门，夹爪停留在约14.2mm间隙，不能靠等待称为抓稳。

S23 将左臂任务专用 PD 从80000/4000改为160000/5656.85424949238，effort及原资格门不改；part0 改用真实侧面抓取位置及相应空手接近路径。part5 实际双指间隙约146/127µm、part0约126/152µm，均完成抓取和安装。原九件 criteria、183阶段及完整动作全部完成，零超时/恢复。正式严格后审仍为false：仅原被动报告覆盖一项失败，运行胶囊事件6752条只保留5000条，阶段胶囊事件3376条未有明确几何消解。原 S23 报告和失败结论不回写。详见 [S22/S23 复盘](../../outputs/stool_assisted_20261021/grant03/S22_S23_pair_review_20261027.json)。

S23 右臂取得资格后，从native3658至56841连续53184 tick覆盖全部166个原左臂阶段；两指间隙约149/209µm，无原主样本失格，实际SDK同钟联接通过。末右臂开爪12 tick、九件独立终态72 tick均完成。原世界支撑保留，`force_support_claimed=false`；这是辅助路线下的持续几何扶持，不证明右臂独立受力承重。原任务没有新增最终双臂回位门。

S24 的物理控制、抓取、目标和阶段计划与 S23 相同，只补原被动报告的完整事件及可独立复核的几何见证。S24重复完成原九件及完整183阶段，14项独立后审全部通过；原被动事件全部保留，并以同钟原生状态和完整关节图逐条复核网格正分离。持续扶持53184 tick覆盖全部166个左臂阶段，末右开爪12 tick及九件终态72 tick均通过。物理控制方案沿用S23，辅助世界支撑仍保留，不宣称右臂独立承重。 具体门、时钟、身份和耗时见 [S24 复盘](../../outputs/stool_assisted_20261021/grant04/S24_round_review.json)。

S25 是保留 S24 后的每步提速试验，动作与原门不变。局部几何查询更快，但同物理帧区间的 part3 搬运仅快约0.87%，底座约慢1.06%，因此主动提前收尾；原 criteria 为2/9，进入43个编译阶段，停在 part7 搬运，零超时/恢复。原主结果、完整动作和严格后审均保持false，不能称完整装配成功。S25 原片已下载核验，工作控制恢复为已通过的 S24。轮次、暂停和剩余额度仍只读唯一账本；[本轮复盘](../../outputs/stool_assisted_20261021/grant05/S25_round_review.json)、[录像核验](../../outputs/stool_assisted_20261021/grant05/deliveries/S25_download_verification.json)及 [S23旧日志无损归档恢复映射](../../outputs/stool_assisted_20261021/grant05/review/cleanup/S23_raw_archive_receipt01.json)保留实际边界。

## 报告修复的范围

保留原胶囊分类、距离、事件总数及全部原事件；Stool 运行报告取消5000条摘要上限。仅对已观察到的四类“腕/前臂端点胶囊—optical board”事件附加完整端点刚体矩阵和时钟。独立后审用同钟原生 raw、原完整关节图和原阶段current/target全12q重建候选，再以完整可见及已声明碰撞端点网格相对板面的正分离平面逐条解释原代理事件。

未知对象对、缺失/截断/过期证据、矩阵或关节图不符、不能证明正分离均拒绝；不以忽略告警或放宽碰撞门换通过。这是所列完整网格的充分分离证明，不宣称连续CCD、PhysX cooked/SDF或接触力已独立验证。S24 前两条实际调用链21项及28条原左臂阶段候选重建已通过，均为零物理检查，不代替正式运行。

## 原件、录像及清理

- 服务器独立源码：`/home/user/beam_integration_20261010_upstream6f57dae/stool_assisted_20261021/stool_repository`；相邻 `tools/`、`runs/`、`baseline/` 保留入口、原件、身份和回退。
- 任务 recipe：`roboassemblybench/tasks/fabrica_stool_circular_ur5e_staged/recipe.yaml`；运行前加载相邻 `tools/stool_env.sh`。Isaac Sim及部分共享资产位于外部，单独复制源码不等于搬走完整运行环境。
- 正式入口仍为 `tools/launch_stool.py`、`tools/close_stool_run.py`；每轮 source/tool/config binding独立冻结。S24 auditor另有部署SHA，不能因其不在 runtime CONTROL_TOOLS 中就声称全部工具未改。
- [S24 三路原片下载核验](../../outputs/stool_assisted_20261021/grant04/deliveries/S24_download_verification.json)逐项记录全部原件SHA，复用服务器唯一一次完整解码；大原报告保留在已核验归档中。原片和关键三视图片段的用途及遮挡限制见 [视觉审阅](../../outputs/stool_assisted_20261021/grant04/review/media_clips/S24/visual_review.json)。画面不能替代原生终态或受力证明。
- [清理汇总](../../outputs/stool_assisted_20261021/grant04/review/cleanup_20261027/cleanup_aggregate_after_S24.json)索引逐项清单和收据。仅删已核验原片可重建的非锚点PNG；旧大日志先无损压缩并完整解压比对SHA，再删明文副本。原报告、raw、录像、失败锚点、回退和源身份保留。旧日志原路径如已归档，后续旧读取器须先从收据映射无损恢复；不得据路径缺失改写历史结论。

## 明确恢复后的入口

最新优化请求、S24 封存和按差异验证入口见 [G05 优化记录](../../outputs/stool_assisted_20261021/grant05/optimization_plan.json)。只按唯一账本读取当前轮次与授权，不叠加旧 G04 余额；原始阶段记录、模型/CPU耗时、严格后审及录像各自给结论。S24 完整封存保留，正在运行的源码不能被后续候选覆盖。

先读唯一账本中 `next_action` 及最新复盘列出的未解决门；未有新明确恢复指令不启动下一轮。复用身份和输入域未变的检查，仅重验改变域；不再重复整个旧预检集合。准备超过30分钟说明实际卡点，不继续堆检查；新瓶颈或连续两次正式尝试无改善提前复盘。完整双臂/夹爪/持件扫掠、原报告覆盖、同钟三视图和缺测处理仍必需。
