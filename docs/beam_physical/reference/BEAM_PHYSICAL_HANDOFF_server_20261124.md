> 原文副本（2026-11-26 从 pcl-server 与 Mac 工作区取得，两处 SHA-256 相同）。文中链接和 `$R`、`$B`、`$S` 等路径指服务器或 Mac 工作区，在本仓库中不一定能点开。复现步骤以同目录上一级的 HANDOFF.md 为准。

# Beam 纯物理装配 · 新 agent 接手指南

**2026-11-24 实查。当前生产版本 v239。第62轮（v239_unload）修复成功：完整装配、叠件静止、最终窗口余量恢复。按人类"授权五轮，或修好停止"，批次 [62, 66] 在第62轮后停止（未用4轮），再跑须有新的人类指令。**

**维护约定**　修改本任务的代码、配置、验收口径、权威路径，或产生新的正式运行结果时，在同一次交付中更新本页，写明改动、已验证结论、遗留问题和证据入口。实时运行状态及授权以本任务服务器账本和最新人类指令为准。

## 先用五分钟确认现场

在用户的 Mac 上执行 `ssh pcl-server` 登录实验室 `lab-12`（192.168.251.12，账号 `user`）。本次已用批处理登录验证能读取唯一账本。Mac 需要校园网络或 aTrust。若 SSH 别名连接失败，在 Mac 运行 `python3 /Users/zhaoheng/.codex/skills/server-fleet/scripts/fleet.py connect lab-12`，沿用用户已有的密钥和主机校验配置。登录后在**服务器 shell**执行以下命令。后文的路径均使用这里定义的变量。

```bash
R=/home/user/beam_integration_20261010_upstream6f57dae
C="$R/candidate_repository"
B="$R/grasp_repairs_20261012_20rounds"
Q="$B/round58_v235_release"
P="$B/offline_v235_release"
O="$C/outputs/fabrica_beam_v235_release"
Q57="$B/round57_v234_fast"
Q56="$B/round56_v233_fast"
Q55="$B/round55_v232_window"
Q59="$B/round59_v236_no_window"
Q60="$B/round60_v237_column2"
P60="$B/offline_v237_column2"
O60="$C/outputs/fabrica_beam_v237_column2"
Q61="$B/round61_v238_lift04"
P61="$B/offline_v238_lift04"
O61="$C/outputs/fabrica_beam_v238_lift04"
Q62="$B/round62_v239_unload"
P62="$B/offline_v239_unload"
O62="$C/outputs/fabrica_beam_v239_unload"
P59="$B/offline_v236_no_window"
O59="$C/outputs/fabrica_beam_v236_no_window"
H="$B/agent_handoff_after53"
cd "$R"
df -h "$R"
/usr/bin/python3 - "$B/round_ledger.json" "$Q55/manual_review.json" <<'PY'
import json, sys
from pathlib import Path
ledger, review = (json.loads(Path(p).read_text()) for p in sys.argv[1:])
print({k: ledger[k] for k in ('status','active_round','next_round','paused','native_continuation_authorized')})
print('挂起轮次', ledger['current_batch']['unstarted_rounds_on_hold'])
print('第55轮终态', review['metrics'], review['terminal'])
PY
```

2026-11-19 第58轮复核后应读到 `status=round58_reviewed_batch_exhausted`、`active_round=None`、`next_round=59`，批次 `[54, 58]` 已用完；第56、57、58轮行 `reviewed_closed`，第55轮行 `assembly_sequence_completed=True`；账本有 `final_support_load_approval`（自第56轮起）。以后以现场账本和最新人类指令为准。旧交接的状态已随原文归档。需要历轮路径时可打开[本机服务器索引](</Users/zhaoheng/Desktop/PCL work/Beam修复/physical_repair_cc_20261114/handoff_after53/server_index.json>)，也可在服务器执行 `/usr/bin/python3 -B "$H/index_server.py" | less` 生成实时索引。

## 从哪里继续查

**2026-11-20 离线复查（人类指令"先离线查清是哪一项提速让第2件压到了底座 解决这个不稳定的问题 然后用轻量输出 最后进行一次仿真"）。** 结论见 FINDINGS P2026-11-20b：离线数据不能把立柱 2 的差异归到单项提速；立柱 2 与底座的接触在两轮都抖，决定性时刻是交接合爪后立柱悬空（第55轮）还是压底座（第59轮）。证据 `$B/analysis_column2_20261120/`，本机 `physical_repair_cc_20261114/v237_column2/analysis/`。修复方向待人类选择，尚未改代码。

**第62轮结果（`$Q62/manual_review.json`）：修复成功，批次停止。** 01:11:13 启动，03:09 结束，60,524 步、6,875 s（8.80 步/秒），`success-criteria-met`，超时 0、恢复 0。微抬前与第61轮逐行相同。立柱 2 微抬后升 0.151 mm、交接后 0.103 mm，对底座从微抬到零件 0 持件检查一直 0 N（预测 ~0.15 / ~0.1 mm，相符）。最终验证第 240 步即通过（第59轮 576 步）；窗口漂移零件 0 0.109 mm / 0.0012 rad、零件 2 0.029 mm / 0.0006 rad（第59轮 1.40 mm / 0.0286、1.43 mm / 0.0294）；五件距目标 ≤0.27 mm、≤0.0042 rad；底座压力 2.39 N、装配总合力 11.93 N。两臂离开后零件 0 每 8 步位移 0.015–0.02 mm（第59轮 0.22–0.38、第55轮 0.006–0.012），角速度均值 0.26 rad/s（第55轮 0.29）。修好判据四条中三条满足；第二条未满足：持件检查时（两手仍夹着）零件 0 对立柱 2 侧向力 4.1 N（第59轮 3.4、第55轮 0），倾斜 6.1 mrad（第59轮 9.2、第55轮 3.2）；松手后降到 0.26 N，再稳定在 ~1.1 N（第55轮 ~1.0 N），最终状态与第55轮一致。对照数据 `$P62/analysis/fixed_all.json`（`fixed_criteria.py`），三路视频 SHA 在 manual_review，本机 `physical_repair_cc_20261114/v239_unload/round62_videos/`（SHA 已核）。
- 若要继续消除持件检查时的 4 N 侧向力：让零件 0 插入目标对齐立柱 2 实测 xy（需改控制器），需新的人类指令。

**2026-11-24 旧输出清理（人类"把过去归档的、未归档的beam纯物理…输出都删除了，保证纯物理装配最新的结果在，并且能够复现就行"；随后"beam辅助路线的结果先别删"、"mac本机上可以一起处理了"）。**
- 服务器（不可恢复）：在第59–61轮及更早的 `$C/outputs/fabrica_beam_*`、`$B` 各轮/离线/批次/性能/归档目录、`$R/runtime_repairs_2026101{1,2}` 中，删除 >2 MiB 的文件 1,574 个（含历次 `.gz` 归档与视频），小文件（脚本、配置、审阅、授权、账本、收据）全部保留。仍被生产代码或第62轮文件精确引用的 8 个大文件未删（如 `batch34_38_20261111/joint_geometry/scene_geometry.json`）。空闲 46 → 66 GiB。计划器与逐文件收据 `$B/cleanup_20261124/`（`plan_summary.json`、`delete_receipt.jsonl`、`execution.json`）。旧轮次的大 trace、视频和第55/59轮对照数据已不存在，以前各段引用的这些路径只剩小文件。
- 保留：`$Q62`、`$P62`、`$O62`（第62轮全部输出与三路视频）、`$B/batch62_66_20261124`、源码、资产、账本。辅助路线 `$R/beam_assisted_20261015` 及其他任务未动。随后（人类"Delete them too"）再删旧轮次 PNG 帧 15,768 个、4.7 GiB（内容与 MP4 相同，第62轮的帧保留），空闲 71 GiB，收据 `$B/cleanup_20261124/png_delete_receipt.jsonl`、`png_execution.json`。
- 复现核对 `$B/repro_check_20261124/`：复制第62轮启动链，生产 preflight 重新生成的 worker 配置 SHA 与第62轮相同（`3bf5d839…`）；420 步 smoke 的 admission 全过，`idle_hold_trace` 18 行（含实测关节角）和 `video_frame_clock` 51 行与 `$O62` 逐行相同，进度行除 smoke 截止行外相同。脚本 `$B/cleanup_20261124/setup_repro_check.sh`。
- Mac：同规则外加 PNG 帧，Beam 纯物理旧输出（`RoboAssemblyBench/outputs/fabrica_beam_*` 及 `beam_grasp_repairs_20261012` 等旧轮目录、`physical_repair_20261110`、`physical_repair_cc_20261114` 除 `v239_unload` 外）共 21,055 个文件、20.7 GiB 已移到废纸篓 `~/.Trash/Beam纯物理旧输出_20261124/`（保留相对路径，清空废纸篓前可放回）。收据和脚本在 `physical_repair_cc_20261114/cleanup_20261124/`。辅助路线、其他任务、`RoboAssemblyBench-Archive`、`backup_preparation_20261029`、`RoboAssemblyBench/submission` 未动。

**第62轮方案与分析（v239_unload，2026-11-24 01:11 CST 启动，已关闭）。** 分析脚本 `$P62/analysis/{insert_end,zstack,lateral,unload_rise}.py`，本机 `physical_repair_cc_20261114/v239_unload/analysis/`。
- 高度：第0件插入期间立柱 2 的高度第55轮 −0.005 mm、第59轮 +0.017 mm，两轮基本相同；零件 0 最终都落在 −0.10 mm。第60/61轮微抬把立柱抬高了 0.95 / 0.35 mm，反而制造了插入吸收不了的高度误差（第60轮左指 51.5 N）。此前"第55轮立柱悬空在上方"的说法不对，悬空的意思只是它对底座的力为 0 N。
- 水平：零件 0 被伺服到同一名义 xy（插入末 (−0.15, −0.08) / (−0.20, −0.07) mm，均带 3.2 mrad 倾斜）；立柱 2 在第55轮偏 (−0.09, −0.53) mm、第59轮偏 (−0.20, +0.29) mm，两轮错位都约 0.4 mm。
- 机理：第55轮立柱对底座 0 N，挂在右手的柔顺夹持（500 N/m）上，零件 0 对中时能侧向让位；第59轮立柱压底座 2.3 N，被摩擦锁住，零件 0 侧向卡住（3.4 N，倾斜 3.2 → 9.2 mrad），之后持续摇晃。
- 卸载量：升 0.1 mm 时底座力已读 0 N（第60/61轮）；右手接手、左手松开后立柱下沉 0.056 mm；带件左手 TCP 稳态比指令低 0.255 mm（第60/61轮一致）。
- 改动：微抬 0.2 mm（预测实际升 ~0.15 mm，交接后 ~0.1 mm，底座 0 N），完成容差改为 max(抬高/2, 0.4 mm)（默认 1 mm 时仍 0.5 mm）。诊断恢复为第59轮设置（第 45,000 步起每 8 步完整接触/物理状态，夹持 trace 开）。`$P62/config_check.json` 通过：不设或 0.001 与第60轮逐字节同；0.0004 等于第61轮配置只改容差；0.0002 只改 offset 和容差。预检配置 SHA 3bf5d839 与离线一致，冒烟与第61轮逐行相同。
- 修好判据：完整装配成功；持件检查时零件 0 对立柱 2 侧向力 < 1 N；两臂离开后零件 0 不持续摇晃；最终窗口零件 0/2 漂移明显低于 2 mm / 0.03 rad。
- 备选（若仍侧向卡）：零件 0 插入目标对齐立柱 2 实测 xy（需改控制器）；若落座正常但叠件仍晃：查最终松手。
- 另：2026-11-21 22:31 有一次第三轮归档（人类"按方案清理"，`$B/archive_20261121/`）：删除第54/57/58轮 contact 与 physics_boundary trace，其余 >20 MB 文件 gzip；第55/59轮只 gzip。读第55轮 trace 需 gzip 读取。

**第61轮结果（`$Q61/manual_review.json`）：0.4 mm 微抬达到想要的状态，但完成条件设得过紧，微抬阶段 720 步超时。** 21:52:19 启动，23:25 结束，46,650 步、5,188 s（8.99 步/秒），输出 1.15 GiB。微抬前与第60轮逐行相同。立柱 2 对底座全程 0 N，相对目标 0.476 mm 并保持稳定（第55轮悬空约 0.45 mm，第60轮 1.00 mm）。带件左手 TCP 稳态比指令低约 0.26 mm：第60轮目标升 1.207 mm、实测 0.951 mm，误差 0.261 < 容差 0.5 → 完成；第61轮目标 0.607 mm、实测 0.355 mm，误差 0.258 > 容差 0.2（抬高量的一半）→ 一直不完成。证据 `$Q61/review/lift_track.{py,txt}`，视频本机副本 `physical_repair_cc_20261114/v238_lift04/round61_videos/`（SHA 已核）。
- 建议修法（未做，需人类指令）：完成容差改为 max(抬高量/2, 0.4 mm)。默认 1 mm 时仍为 0.5 mm，与第60轮逐字节相同；0.4 mm 时为 0.4 mm，第61轮实测误差 0.258 mm 即可完成（要求至少升 0.2 mm）。

**第61轮（v238_lift04，2026-11-21 21:52 CST 启动，已关闭）。** 人类"改 0.4 mm 再跑"，授权 `$B/round61_20261121/human_authorization.json`。编译器只把微抬高度改为可配：`BEAM_HANDOVER_UNLOAD_LIFT_M`（默认 0.001 即第60轮，范围 0.0002–0.001 m），完成容差为抬高量的一半；本轮 0.0004 m（0.4 mm，容差 0.2 mm）。`$P61/config_check.json` 通过：不设或设 0.001 与第60轮配置逐字节相同；设 0.0004 只改该阶段 offset 和 position_tolerance，`max_steps` 不变；越界或非数字被拒。预检配置 SHA a5989b97 与离线一致，420 步冒烟与第60轮逐行相同。轻量输出同第60轮。回退件 `$P61/compiler_before61.py`，本机副本 `physical_repair_cc_20261114/v238_lift04/`。可检：微抬前与第60轮逐行相同；立柱 2 对底座约 0 N、相对目标误差约 0.4–0.5 mm；零件 0 插入及持件检查左指力 < 40 N；最终验证通过且零件 0/2 余量宽于第59轮。

**第60轮结果（`$Q60/manual_review.json`）：微抬生效，但零件 0 插到底时顶住立柱 2，左指力超限失败。** 11:55:55 启动，13:49 结束，57,583 步、6,389 s（9.01 步/秒），超时 0，禁止的状态写入 0。新阶段前与第59轮逐行相同。微抬后立柱 2 对底座一直 0 N（第59轮 2.3 N），相对目标误差 1.00 mm、右手夹住时 0.98 mm（第59轮 0.49）。零件 0 插入时左指力 20.9/21.2 N，`insert_09` 末尾升到 24.0/19.5 N，`assemble_03_part_0_physical_hold_check` 第 2 步达 51.5 N（限 40）→ `local-skill-failure:loaded_gripper_force_limit`。原因：零件 0 按名义装配位插入，立柱 2 却高出约 1 mm，帽件提前 1 mm 顶到立柱，把左指压进力限。第55轮立柱悬空时误差只有 0.45 mm，1 mm 抬得过多。轻量输出：1.46 GiB / 57,583 步（第59轮 3.31 GiB / 60,579 步）；最大项为 episode 与 collect_results 各 222 MB、motion 123 MB、assembly_integrity 与 table_support 各 106 MB，contact 15 MB，物理状态 44 MB。证据 `$Q60/review/r60_fail.{py,txt}`，三路视频 SHA 在 manual_review 中，本机副本 `physical_repair_cc_20261114/v237_column2/round60_videos/`（SHA 已核）。
- 下一步候选（未做，需人类指令）：抬高量改为约 0.3–0.5 mm（第55轮悬空量级）；或零件 0 插入前让左手把立柱放回、右手只夹不托；或让零件 0 的插入目标跟随立柱 2 的实测位姿。

**第60轮（v237_column2，2026-11-20 11:55:55 CST 启动，已关闭）。** 人类选"交接后微抬 1 mm"，授权记录 `$B/round60_20261120/human_authorization.json`。
- 编译器只加一处，开关 `BEAM_HANDOVER_UNLOAD_LIFT=1`（`_apply_handover_unload_lift`）：在 `handover_left_2_close_and_attach` 之后插入 `handover_left_2_unload_lift`，左手以 grip_contact_prime 的带件竖直伺服把立柱 2 上抬 1 mm（每次 0.1 mm，测得 TCP 距目标 0.5 mm 内算完成，min 72 步，超时 720），右手继续夹住立柱 3。目的：右手接住立柱 2 时它是悬空的（第55轮状态）。`$P60/config_check.json` 通过：不设或设 0 与第59轮配置逐字节相同；设 1 只多这一阶段及其派生注记，`max_steps` +721（超时 720 加每阶段 1 步），最终验证不变。回退件 `$P60/compiler_before60.py`。
- 轻量输出（人类"用轻量输出"）：探针/视图类观察器照旧；周期性完整接触记录关闭（`BEAM_OPT_FULL_TRACE_FROM_STEP=-1`，只留失败快照，探针相同）；`hold_trace.jsonl` 是指向 /dev/null 的符号链接（不能直接设 /dev/null，loaded_force 等按它的目录写）；物理状态轨迹改为全程每 480 步一条（原为第 45,000 步起每 8 步）。motion 及其衍生 trace（assembly_integrity、table_support、seat、ik 等）保留。
- 冒烟与第59轮逐行相同，预检配置 SHA cb57b2f9 与离线一致。`launch_round60.py` 原有"账本里已有完整成功就不启动"的旧断言（第54–58轮成功即停规则），改为"之前的成功只有第59轮，且本轮授权为 2026-11-20 人类答复"，改动和旧 SHA 记在 `$Q60/launch_identity_note.json`。
- 预期可检：新阶段前与第59轮逐行相同；微抬后至零件 0 期间立柱 2 对底座约 0 N；右手夹力接近 8 N；最终窗口零件 0/2 漂移明显小于 2 mm / 0.03 rad；输出明显小于 3.3 GiB。本机副本 `physical_repair_cc_20261114/v237_column2/`。

**第59轮结果（`$Q59/manual_review.json`）：首次正式完整装配成功。** 2026-11-19 22:05:28 启动，23:59:57 结束，退出码 0，60,579 步、6,795 s（8.92 步/秒），超时 0、恢复 0，初始化后禁止的状态写入 0。第 57,676 步前与第58轮逐行相同。零件 0 退让 443 步、最后松爪 101 步（min 72 + 30），两臂回原位后进入最终验证，第 576 步通过（固定参考窗 240/240，有过重置）。底座压力 9.89 N，装配总合力 11.20 N（≥9.81）。五件距目标：零件 6/3/2/1/0 为 0.15/0.38/1.23/0.41/0.97 mm，姿态 0.0014–0.0269 rad（限 3 mm / 0.04 rad）。窗口内漂移：零件 2 1.43 mm / 0.0294 rad、零件 0 1.40 mm / 0.0286 rad，接近 2 mm / 0.03 rad 上限，余量很窄。原始速度单独报告（第51轮人类批准口径）：零件 0 结束时仍在抖（0.017 m/s、1.95 rad/s），零件 2 0.80 rad/s，原始静止检查只有底座通过。到最终验证比第55轮早 15,945 步（约 21%）；叠件偏航（相对底座）第3件 / 帽件1 为 4.31 / 4.34 mrad。三路视频 SHA 记在 manual_review 中，本机副本 `physical_repair_cc_20261114/v236_no_window/round59_videos/`（SHA 已核）。
- 遗留：零件 0、2 在叠件上的接触抖振（v234 提速改变了立柱 2 的夹持状态，见下）使最终窗口余量只有约 2%（姿态），可复现性未知；未定位到具体哪一项提速。

**第59轮（v236_no_window，2026-11-19 22:05 CST 启动，已关闭）。** 授权记录 `$B/round59_20261119/human_authorization.json`（人类答复"好的帮我启动吧……"），`prepare_round59.py` 已开批次 [59, 59]、`maximum_rounds` 59，启动时空闲磁盘 53.1 GiB、内存 25.0 GiB。接手时先读账本行 59 和 `$O59/exit_code`；结束后用 `$Q59/prefix_check.py` 对照第58轮，再按第58轮的方式复核关闭。 人类 2026-11-19 看第57轮视频后认为零件已稳定放上，问"如果不管这些稳定门测验，直接退开手，装配好的东西也不会散吧？……可以这样试试吗？"。复查结论：
- 原因（`$Q/review/support_chain.{py,json}`）：零件 0 坐在由右爪夹着的立柱 2 上。第0件下放前两轮就不同：第55轮立柱 2 压底座约 0.2 N、右爪两指 8.1/6.9 N；第58轮 2.3 N、5.1/5.8 N。第58轮零件 0 插好时对立柱 2 的侧向力 3.4 N（第55轮 0）。松手后立柱 2 对底座的力在 ±2 N 内来回跳，零件 0 一直摇晃；右爪松开、没有任何手指接触后也不停，像接触求解抖振。来源是 v234 对零件 3/2/1 的提速改变了立柱 2 的夹持状态，具体哪一项未隔离。
- 最终验收回放（`$Q/review/final_gate_replay.{py,json}`）：用第58轮实测样本按最终窗口（2 mm / 0.03 rad、任一超限全部重置）回放：退让 616 步、停靠 208 步、失败后两臂静止 1,197 步均零重置；最后松爪中 1 次（零件 0 2.02 mm），最长 504 步。零件 0 离目标最大 1.65 mm（限 3 mm）。不会散，最终窗口大概率能过，但零件 0 余量窄（1.3–2.0 mm 对 2 mm）。
- v236 只改编译器一处，开关 `BEAM_RELEASE_WITHOUT_STACK_WINDOW=1`（需同时开 v235 的开关，`_apply_release_without_stack_window`）：`assemble_03_part_0_retreat` 回到只看竖直退让完成，超时 2,400 → 1,200；`hold_2_final_release` 去掉五件 0.5 mm 窗，只留零件 2 到位（15 mm / 0.12 rad，min 72 后连续 30 步）。最终无负载验证一条不改。`$P59/config_check.json` 通过（不设或设 0 与第58轮配置逐字节相同；设 1 只改这两处和 `max_steps` −1,200；未开 v235 开关时拒绝），预检配置 SHA da5643fa 与离线一致，420 步冒烟与第58轮逐行相同。回退件 `$P59/compiler_before59.py`。
- 启动顺序：人类确认后写 `$B/round59_20261119/human_authorization.json`（instruction、human_answer、at_utc）→ `python $Q59/prepare_round59.py`（开批次 [59, 59]、`maximum_rounds` 59、行 59 `ready_for_native`）→ `python $Q59/launch_round59.py`。实时对照用 `$Q59/prefix_check.py`（第 57,676 步前应与第58轮逐行相同）。本机副本 `physical_repair_cc_20261114/v236_no_window/`。
- 磁盘（2026-11-19 22 时）：服务器根分区 491G，已用 412G、空闲 54G（89%）；本项目 `$R` 共约 51G，`/home/user` 共约 332G。第54–58轮新增约 15.3 GiB：输出 3.8/3.5/0.3/3.2/3.3 GiB（第56轮在落座处早停），轮次目录合计约 1.2 GiB；单轮完整运行约 3.3 GiB，最大的是 physics_boundary_trace（0.84 GiB）、contact_trace（0.53 GiB）、hold_trace（0.43 GiB），episode 与 collect_results 各 0.22 GiB 且内容重复。空闲从第54轮启动前 27.8G 降到第56轮 19.3G，两次归档腾出约 44.8G，第58轮启动时 57.1G，现 54G。
- 第三次归档（2026-11-21，人类"按方案清理"）：第54/57/58轮（v231/v234/v235）删 contact_trace.jsonl、physics_boundary_trace.jsonl，其余 >20 MB 文本 gzip；第55/59轮（v232/v236）只做无损 gzip；v233、v237、v238 未动；视频和小文件保留。删 6 个、压缩 55 个（解压核 SHA 后才删原文件），腾出 13.9 GiB，结束时空闲 56.0 GiB（2026-11-24 实查 51G）。收据 `$B/archive_20261121/archive_rounds54_60_receipt.json`，脚本本机 `physical_repair_cc_20261114/v238_lift04/archive_rounds54_60.py`。读这些轮的轨迹需 `gzip -dc`。

**第58轮结果（`$Q/manual_review.json`）：零件 0 松爪后流程推进三步，最后一步松爪超时；零件 0 坐不稳。** 第 57,575 步前与第57轮逐行相同。`assemble_03_part_0_release` 101 步通过；`assemble_03_part_0_retreat` 第 622 步通过，五件窗靠一段 120 步的安静期过关（13 次重置，均由零件 0 触发）；停靠 216 步通过。`hold_2_final_release` 720 步超时（第55轮 101 步、零重置）：五件窗重置 10 次，每次都有零件 0，其中 2 次还有零件 2。59,234 步 6,747 s（8.78 步/秒），到这一步比第55轮早 15,082 步。
- 零件 0 在左臂退走、停靠后仍持续摇晃：每 8 步平均位移在退让、停靠、最后松爪中依次为 0.35、0.38、0.27 mm（第55轮 0.13、0.006、0.023），原始角速度约 1.5–1.9 rad/s。失败后录制中两臂不动，仍有 0.31/0.41 mm、最大 1.28 mm。所以第57轮"摇晃靠夹爪维持、退让后会停"的推断不成立。
- 零件 0 落座更差：物理持件检查时相对误差 0.58 mm（第55轮 0.19），最后松爪中 0.32–1.65 mm、平均 0.93（第55轮 0.50–0.52，静止）。零件 0 各阶段参数没变，但起点状态被 v234 提速改变（叠件偏航、取件时机），来源尚未隔离。最终验证要求 240 步内线速度 ≤0.01、角速度 ≤0.1，按现状也过不了。
- 证据：`$Q/review/final_release.{py,json}`（第55/58轮的窗口重置和各件运动）、`$Q/review/part0_seat.{py,txt}`，三路视频 SHA 记在 manual_review 中。本机副本 `physical_repair_cc_20261114/v235_release/round58_review/`、`server_records/`。

**第58轮（v235_release，2026-11-19 16:48:42 CST 启动，18:43 结束，已关闭，输出 `$O`）。** 相对 v234 只改编译器一处，开关 `BEAM_PART0_RELEASE_AFTER_CLEAR=1`（`_apply_part0_release_after_clear`）：
- `assemble_03_part_0_release` 只要求零件 0 在 `part_0_assembled` 上（15 mm / 0.12 rad，min 72 步后连续 30 步）；张爪脱附、固定臂指令不变。
- 五件固定参考位姿窗（0.5 mm / 0.0087 rad、原始 0.05 m/s / 2 rad/s、min 72、连续 30 步）原样移到 `assemble_03_part_0_retreat`，与竖直退让完成一起判定；退让超时 1,200 → 2,400 步（`max_steps` +1,200）。
- `$P/config_check.json` 通过：不设或设 0 时与第57轮配置逐字节相同；设 1 只改这两个阶段和派生的 `max_steps`，阶段合同审查无问题。预检、420 步冒烟（与第57轮逐行相同）通过，`$Q/candidate_review.json` 已写。支撑状态模块、worker、获批的最终合力门和速度缓存都与第57轮相同。
- 第 57,575 步（零件 0 松爪）前应与第57轮逐行相同。用 `$Q/prefix_check.py` 实时对照第57轮，并报告相对第55轮的提前量、叠件偏航和最终验证中的合力。
- 回退件 `$P/compiler_before58.py`。本机副本：`physical_repair_cc_20261114/v235_release/`（`source/`、`round58/`、`round57_review/`、`test_config.py`）。

**第57轮结果（`$Q57/manual_review.json`）：提速生效，五件全部装入；零件 0 松爪超时。** 到 `assemble_00_part_3_descend` 前与第55轮逐行相同，底座落座也复现。第3、2、1件的下降、搬运和 `insert_00` 提速全部生效，零件 0 松爪比第55轮提前 15,608 步（57,575 对 73,183），抓取、滑移、力带、接触门全过，58,296 步 6,579 s（8.86 步/秒）。`assemble_03_part_0_release` 720 步超时，第55轮在第 475 步通过：两指第 2 步起无力，零件 0 在叠件上摇晃约 ±0.5 mm、1–3 rad/s，每 16–50 步让五件共用的固定参考位姿窗重置，第55轮只是碰上一段安静窗口。失败后 1,200 步录制中臂不动、夹爪张开，摇晃不减；第55轮在竖直退让第 182 步摇晃停止。证据 `$Q57/review/release_rattle.{py,json}`，三路视频 SHA 记在 manual_review 中。

**第57轮（v234_fast，2026-11-19 14:28:59 CST 启动，已关闭，输出 `$C/outputs/fabrica_beam_v234_fast`）。** 相对 v233 只改编译器一处：`_FAST_CARRIES` 去掉 `base_6_assembly_clearance`，底座搬运回到 0.04 m/s / 0.2 rad/s。支撑状态模块和 worker 与第56轮相同。
- 旧输出归档（2026-11-19，人类指令"对以后诊断没用的直接删除，可能有用的无损压缩"）：删除 v201–v227 的 `contact_trace.jsonl.gz` 和 v201–v230 的逐帧 PNG（`episode_0000_live_frames/`，同内容在 MP4 中）；其余 >20 MB 明文无损 gzip，校验后删除明文；v229/v230 接触 trace、视频、关键帧、小文件保留；v231–v233 未动。腾出 30.6 GiB。逐文件 SHA 收据 `$B/batch54_58_20261119/archive_old_outputs_receipt.json`，删除件不可恢复。第二轮（人类"Delete per your advice"）：再删 v229/v230 接触 trace、v201–v227 物理 trace 与关键帧、v201–v230 实体几何和抓取扫掠 trace，腾出 11.1 GiB，空闲 60.6 GiB；收据 `archive_old_outputs2_receipt.json`。保留各轮视频、压缩摘要、小文件及 v229/v230 物理 trace 和关键帧。
- 启动命令（第58轮已用过，不能重放）：`python $Q/launch_round58.py` 会核资源，空闲磁盘不足 19 GiB 时只打印 `blocked_by_current_resource_admission`，不启动。
- 磁盘：第58轮启动时空闲约 57 GiB，结束后 54 GiB；第54轮输出未动。

**第56轮结果（`$Q56/manual_review.json`）：底座落座超时。** 前 28 行与第55轮相同，底座快搬少 1,789 步、持件全过；`base_6_set_down` 4,800 步超时（11,571 步，11.46 步/秒）。落座控制器要求底座原始角速度 ≤0.5 rad/s，第56轮夹持中的底座读数接近时 0.45–0.56、接触后 0.7–0.9 rad/s（第55轮 0.09–0.27），只加载 1 步，压力停在 0.2–0.3 N。原因是快转让抓力调节把两指力提到 10.9 N（第55轮 9.6/9.8 N）并保持；抓取关系不变。其余零件后续门槛为 2.0–6.0 rad/s。对比脚本 `$Q56/review/seat_compare.{py,json}`。

**第56轮（v233_fast，2026-11-19 13:27:50 CST 启动，已关闭）。** 在 v232 上新增两个编译开关和一个 worker 补丁，其余判据、目标、路径、扶持顺序和求解器不变：
- `BEAM_FINAL_ASSEMBLY_LOAD=1`（人类批准，记录见上）：最终 `beam_table_supported` 保留 9.81 N，但量的是零件 6、3、2、1、0 对支撑物的合力。只读已有的接触探针，同一物理帧内读两次时钟，任何探针缺失、张量无效或时钟变化都判不通过；底座6 自身 ≥1.962 N（它落座时的门槛），底面间隙 −0.5…1.0 mm 不变；legacy 诊断仍按只量底座的 9.81 N。按第55轮数据每个样本都满足（合力 11.62–12.22 N）。
- `BEAM_FAST_MOTION=1`：第3、2、1件及换手、扶持臂的 6 段空臂下降套用零件 0 的分段超前；底座、第2件、帽件1 的搬运 0.04 → 0.10 m/s、0.2 → 0.4 rad/s；第3件只提线速度，转动仍限 0.20 rad/s（编译器对它有 ≤0.20 的校验）；第3、2、1件 `insert_00` 在离路点 45 mm 外加快。这些搬运要转 1.6–3.1 rad，所以主要是角速度在限时间。
- worker `BEAM_OPT_VELOCITY_CACHE=1`：纯读取的 `_object_velocity_metrics`（profile 中占约 22%，每步约 24 次）按物理帧缓存。A/B 跑到第 4,700 步，进度行和 hold/motion/attach/close-gate trace 都与第55轮逐行相同，2,000–4,600 步 90.4 → 82.1 ms/步（约快 10%）。profile 在 `$B/perf_r56_20261119/profile_v232/`，A/B 结果 `$B/offline_v233_fast/velocity_cache_ab.json`。
- 完整接触/物理 trace 改从第 45,000 步开始（第55轮为 65,000）。
- 离线检查在 `$B/offline_v233_fast/`：`config_check.json`（两开关不设或设 0 时配置与第55轮逐字节相同；设 1 只改 16 个阶段的目标键和最终支撑条件；非法值被拒）、`gate_check.json`（新门 11 个用例及与旧模块的等价性）。生产预检、阶段合同、420 步冒烟均通过。回退件 `compiler_before56.py`、`support_state_before56.py`、`tools/worker_before56.py`。
- 预计比第55轮少约 14k 步（约 63k 步），每步约快 10%。第一个改动阶段是 `base_6_assembly_clearance`（第55轮第 3,356 步），此前应与第55轮逐行相同。`$Q56/prefix_check.py` 实时对照第55轮，报告各阶段提前量、叠件偏航和最终验证里的合力。
- 未验证：底座、第2件、帽件1 以 0.4 rad/s 转动；扶持臂向已装立柱的分段下降；新门的原生运行。
- 本机副本：`physical_repair_cc_20261114/v233_fast/`（`source/`、`tools/`、`round56/`、`perf/` 及各检查脚本）；第57轮在 `physical_repair_cc_20261114/v234_fast/`。

**第55轮结果（2026-11-19 复核，`$Q55/manual_review.json`）：装配动作全部完成，卡在最终验证的一条力门槛。** 前 360 行与第54轮逐行相同。v232 的零件 0 提速全部生效：下降少 847 步，两段搬运分别少 757 步和 2,719 步，`insert_00` 提前 4,286 步开始。抓取、搬运和接触力门槛都通过。零件 0 在第 73,015 步入座，第2件扶持在第 74,316 步松开，均早于第54轮失稳点（76,904）。随后零件 0 松手、退让，扶持臂退让，两臂回 home，第 75,948 步进入 `beam_final_unloaded_verification`，跑满 1,200 步后 `timeout-failure`。forbidden 写入为 0，exit 0。
- 离线逐条重放最终门（`$Q55/final_gate_replay.py` → `$Q55/review/final_gate_replay.json`）：
  - 五件全部 1,200 个样本都在目标 3 mm/0.04 rad 内，最大误差 0.67 mm/0.0072 rad；底面间隙全在 −0.5…1.0 mm 内。
  - 固定参考位姿窗（2 mm/0.03 rad）连续 1,200 步成立，要求 240 步。
  - 五件均已脱附、未锁、碰撞开着；两爪全开。
  - 原始速度只做诊断，没有一步五件同时 ≤0.01 m/s、0.1 rad/s，这是 2026-11-17 人类批准的位姿窗口径。
- 唯一不过的条件：`beam_table_supported` 要求 `minimum_vertical_force=9.81`（`beam_coordinated.py:333`），而它只量底座6 与支撑物之间的压力。实测均值 3.33 N，最大 6.76 N，没有一步达到。
- 物理原因：五件原生质量各 0.25 kg，共 12.26 N。第3、2件穿过底座直接立在 `assembly_support` 上，支撑板经三件受力：底座6 3.35 N、第3件 3.88 N、第2件 4.73 N，合计 11.95 N（11.62–12.22 N，每 8 步一个样本，150 个），即整机重量都由台面承担。只量底座6，9.81 N 物理上达不到。2026-11-17 人类批准保留"原 force 门"，所以这条判据不能自行改。
- 用户已批准改门（2026-11-19，"改门，并提速"），并选择与提速一起在第56轮做，见上。

**步数与速度预算（`$Q55/step_budget.py` → `$Q55/review/step_budget.json`，按阶段名分类）。** 共 77,148 步，墙钟 8,107 s：
- 带件搬运 32,836 步（42.6%），其中 `assembly_clearance` 18,899 步：底座 3,222，第3件 4,389，第2件 4,405，第1件 5,252，零件 0 只用了 1,631。
- 空臂接近 19,433 步（25.2%），其中下降 10,106 步，`move_above` 5,977 步。
- 抓取/稳定/检查/落座 11,726 步（15.2%，7.42 步/秒最慢，`base_6_set_down` 2,653）。
- 插装 5,474 步，退让/回 home 5,094 步。
- 把零件 0 已验证的搬运、下降和 `insert_00` 参数推广到底座和第3、2、1件，并让最终验证 240 步通过，预计可少约 20k 步（约 26%，墙钟约 −35 min）。`move_above`、闭爪、退让还有余量。

**此前各轮（已解决或已关闭）。**
- 第53轮：零件 0 抓得太浅，提起时 `contact_missing`。第54轮（v231，`BEAM_CAP0_PICKUP_DEPTH_M=-0.0015`）解决。
- 第54轮（`$B/round54_v231_cap_depth/manual_review.json`）：在 `insert_00` 第 77,190 步，无人扶持的第3件+帽件1 自发失稳，第3件相对底座偏航 0.0501 rad > 0.05。CAD 回放排除了碰撞。右臂扶第2件期间底座持续微振（角速度 RMS 0.15–0.29 rad/s），帽件1 在第3件上蠕移。失稳发生在 hold_3→2 换手（第 60,983 步）之后 15.9k 步。
- 第55轮 v232 的改法：开关 `BEAM_PART0_WINDOW_SPEEDUP=1`，在 staged contract 之后单独一遍，只改 4 个零件 0 阶段的 14 个键，判据和路径不变：
  - 下降：离抓取位 50 mm 外用 3 mm 超前，关节超前上限 0.004 → 0.012。
  - 搬运：平滑驱动 0.04 → 0.10 m/s，前瞻 2 → 5 mm，角速度 0.2 → 0.5 rad/s。
  - `insert_00`：离路点 45 mm 外每步累加 0.1 → 0.25 mm。
  - 离线检查和回退件在 `$P`。
- 本机脚本副本在 `physical_repair_cc_20261114/v232_window/`。第55轮三路视频 SHA 记在 `$Q55/manual_review.json`。用户要求后已下载到本机 `physical_repair_cc_20261114/v232_window/round55/videos/`，SHA 已核对一致；`INDEX.md` 列出各阶段的视频时间点（视频按实时播放，秒数 = 步数 ÷ 240）。

## 仿真耗时与日志体积诊断及对照测试（2026-11-18）

第53轮 4 h 22 min / 68,988 步（228 ms/步），输出 6.3 GB；Plumbers C01 为 113 ms/步、157 MB。按人类指令“帮我进行对照测试”，在 `$B/perf_ab_20261118/` 做了前 11,300 步对照（不计正式轮次，账本未改）：

- 关掉日志主要省空间：compat 配置与原配置逐数相同，输出 416 → 121 MB，但只快 5%（150 → 143 ms/步）。全部关掉快 20%，轨迹却改变。
- **部分诊断会改变物理**：接触 trace 加支撑接触同开、grasp-sweep 补丁和被动实体几何监控都会懒创建接触探针，Isaac 因此把被测刚体的 `sleepThreshold` 设为 0。第1–53轮均在这些探针存在的条件下运行。关闭它们会使闭爪提前 44 步，并影响桌上自由件的微小运动。今后比较轨迹，或用 lean 配置做正式运行之前，先读这一点。
- cProfile（compat，跑到第 18,000 步，与第53轮29次转移相同）：物理加渲染只占 9–16%。`get_tracked_object_states` 每步2次，占约 43%；`_physical_hold_valid` 每步约9次；`copy.deepcopy` 约占 40%。提速重点是同帧缓存和去掉深拷贝。
- 运行时补丁对照（按“Yes, go ahead”，只在对照包 `opt_worker.py`，生产源码未改）：实现了三项，即只建探针不建记录、深拷贝改 pickle 往返、`_gripper_contact_metrics` 同帧缓存。跑到第 18,000 步，墙钟 3,055 → 1,885 s（170 → 105 ms/步，省 38%），阶段转移、进度、五件与双臂终态、状态写入与 compat 逐数相同。`get_tracked_object_states` 同帧缓存尚未做。
- 入口：compat 为 `$B/perf_ab_20261118/run_ab.sh original` 加 `BEAM_AB_COMPAT=1`；优化版为 `run_ab.sh opt` 加三个 `BEAM_OPT_*=1`。第 18,000 步以后的等价性和全流程耗时尚未验证；移入生产源码需单独授权。

结果表、逐组分离和后续建议见[性能诊断](beam_routes/physical/PERFORMANCE_DIAGNOSIS.md)。

## 证据、源码和检查代码的服务器入口

| 要找什么 | 去哪里读 |
|---|---|
| 所有轮次与授权 | **`$B/round_ledger.json`** 是唯一运行账本。`$H/server_index.json` 为此次快照，含1–53轮、实际存在的轮目录、历史候选输出和离线目录。早期轮次字段不统一，先看 `contract`、`run`、`output`、`version` 和同号目录。部分轮次为离线检查或启动失败。历史批次授权在 `$B/batch*/`，本批原授权在 `$B/batch52_56_20261117/human_authorization.json`。 |
| 当前生产代码 | `$C` 为工作树。任务编排、阶段配置及纯物理判定从 `roboassemblybench/core/beam_coordinated.py`、`beam_staged_physical.py`、`beam_physical_compiler.py` 开始。任务状态从 `internutopia_extension/tasks/factory_dual_franka_assembly_task.py` 进入。 |
| 生产派发与第53轮改动 | 在 `$C/toolkits/factory_dual_franka_assembly/` 读 `task_specs.py`、`generate_demos.py`、`local_skills.py` 和 `plumbers_block_ur5e_skills.py`。最后一个文件含 `_close_pose_gate_action` 及本轮新增的 `_translation_only_recenter_ik`。`$Q/production_worker_config.json` 给出生产实际选中的阶段、机器人和参数。用 `grep -Rn --include='*.py' 'held_object_not_grasped' "$C/toolkits" "$C/internutopia_extension" "$C/roboassemblybench"` 找到失败判定及其调用者。 |
| 身份与回退 | `$Q/source_manifest.json` 列出900个源文件，本次对当前 `$C` 逐文件验 SHA 全部相符。源清单 SHA-256 为 `757d5e81bf2bdd8540ffdb63b0106919f2288f8926c650c96d5be406fba6a7ef`，worker 配置 SHA-256 为 `363b1f32b20fb9860327ae665a5543c9f1e435bea24eef10c83b1801e0ebd437`。`$O/source_before.json`、`source_after.json` 为运行前后核验。`$P/source/` 是离线候选；`$P/controller_before53.py` 是本次唯一变更文件的部署前副本。部分离线源码树含硬链接，做新实验时先复制为独立文件或原子替换。 |
| 运行包与资源监督 | `$Q/checked_worker.py`、`run_full_assembly.sh`、`launch_round53.py`、`tools/supervise.py` 为本轮冻结的启动和监督代码。`$Q/tools/worker.py` 复用 `$B/round24_native_release/full_assembly_worker.py`。同目录的 `launch_identity_manifest.json`、`tools_manifest.json`、`experiment_contract.json`、`capture_output_contract.json` 记录入口身份、合同与容量预算。旧启动脚本留作取证。 |
| 资产和几何 | `$Q/asset_manifest.json` 给出帽件 USD 原件和哈希；`scene_asset_signatures.json` 记录碰撞网格身份。`geometry_reference_manifest.json`、`close_geometry_asset_manifest.json` 指向 CAD 输入及几何代码。`$B/batch34_38_20261111/joint_geometry/`、`geometry_tools/` 是几何基准，`$R/runtime_tools/geometry_deps/` 存依赖。实际加载结果看 `$O/native_scene_asset_admission.json`、`native_solver_after_reset.json`。 |
| 离线回归与生产预检 | 仓库测试在 `$C/tests/toolkits/test_beam*.py`。本轮离线入口有 `$P/qualify_config.py`、`regression_checks02.py`、`test_R3.py`、`test_final_acceptance.py`、`test_common_window.py`。250项回归的分组输出在 `$P/regressions/result.json`；19项启动保护结果在 `$Q/start_guard_qualification.json`。部署审核在 `$Q/candidate_review.json`、`production_preflight.json`、`phase_contract_preflight.json`。检查实现见 `$Q/tools/preflight.py`、`verify_source.py`、`close_cad_guard.py`、`joint_cad_guard.py`、`state_write_audit.py`。`$P/replay/result.json`、`dispatch.json` 记录停止帧回放。 |
| 检查脚本原文 | `$H/v230_cap_close/` 单独收齐此前从 Mac 派发的本轮实现、检查、准入、取证和媒体脚本，包括 `stage.py`、`replay.py`、`dispatch.py`、`qualify53.py`、`prepare.py`、`read_originals.py`。部分是由 `remote_check.py` 注入路径的历史片段，其中 `O` 在回放中指**第52轮输出**。复跑停止帧使用下文独立临时目录入口，避免覆写旧检查结果。 |
| 第53轮原结果与诊断 | `$O/collect_results.json` 为 summary 列表；`$O/episode_0000.json` 的 `metrics.phase_transition_history` 保存状态转移。`$Q/manual_review.json` 含两份原件各自的大小、哈希和末段事件。`$O/close_gate_trace.jsonl`、`close_control_trace.jsonl`、`hold_trace.jsonl`、`attach_debug.jsonl`、`grasp_sweep_trace.jsonl`、`loaded_force_trace.jsonl` 用于闭爪及提起诊断。`contact_trace.jsonl.gz`、`physics_boundary_trace.jsonl.gz`、`motion_trace.jsonl.gz` 可在服务器流式读。`state_write_counts.json`、`resource_snapshots.jsonl`、`post_failure_recording.json` 分别核查写入、资源和失败尾录。 |
| 视频、照片、时间对齐 | `$O/episode_0000_live_videos/` 有正面、左腕、右腕原 MP4；`$O/episode_0000_live_frames/`、`capture_key_frames/` 有留存帧；`video_frame_clock.jsonl` 记录帧时钟。三路各8,773帧、约4分52秒。完整解码记录在 `$Q/video_decode_review.json`，跨机 SHA 在 `$H/round53_video_review/verified.json`。[本机视频与校验单](</Users/zhaoheng/Desktop/PCL work/Beam修复/physical_repair_cc_20261114/round53_video_review/>) |
| 历史方法及诊断 | 路线发现按 `$R/beam_routes/physical/FINDINGS.md` 检索，通用检查在 `$R/BEAM_REVIEW_CHECKLIST.md`。`$B/offline_v*/`、`round*/`、`batch*/` 分别定位离线候选、冻结轮次和批次级诊断。`$B/cc51_autonomous_20261117/HANDOFF_CC51.md` 是CC自治交接；`$H/historical_cc/` 汇集本机现存的历次指导原文，事实包和协调记录仍在相应批次目录。当前轮次权限以唯一账本为准。 |

## 已验证的离线复核入口

在 **Mac 工作区**执行下面这段。它用项目已有的 SSH helper 加载服务器 `internutopia311` 和 Isaac Sim 5.1.0，读取第52轮失败帧，用第53轮候选运行实际闭爪控制器与 CAD 检查。新输出写入独立临时目录，退出时清除。此次交接实测预测角点位移由 182.794 微米降至 4.999 微米。回放重建了当时待执行的微调状态，覆盖闭爪动作；后续提起动力学仍要结合原生证据判断。

```bash
cd '/Users/zhaoheng/Desktop/PCL work/Beam修复'
/opt/anaconda3/bin/python3.12 -B physical_repair_cc_20261114/v230_cap_close/remote_check.py --isaac <<'PY'
import runpy
runpy.run_path(str(B/'agent_handoff_after53/replay_in_temporary_dir.py'), run_name='__main__')
PY
```

**正式运行和验收。** 环境设置在 `$Q/run_full_assembly.sh` 及本机 `physical_repair_cc_20261114/remote_python.py`。本轮采用 TGS、CPU dynamics、1/240秒物理步，GPU UUID 为 `GPU-e064bda0-b445-2d7f-50d5-c1180deadea2`。本批启动门槛为可用内存12 GiB、GPU显存12 GiB、磁盘19 GiB（含5 GiB保留），运行保护为内存2 GiB、磁盘5 GiB。后续获准开新轮时需重新核现场资源。

五件均须到达目标位姿（3毫米、0.04弧度），真实释放，双臂实际回位。五件须在共同连续240个新物理步内相对固定参考保持稳定（漂移≤2毫米、≤0.03弧度），原始速度独立报告。自第56轮起（2026-11-19 人类批准），最终台面支撑量全部已装件对支撑物的合力（≥9.81 N，≤40 N），底座6 自身 ≥1.962 N，底面间隙 −0.5…1.0 mm；只量底座的 9.81 N 保留为 legacy 诊断。实际启用项见 worker 配置、冻结合同及 `$Q/human_final_acceptance_approval.json`，批准文件保留了最初单轮批准的历史措辞。禁止固定抓取/装配连接、运行时写物体位姿、清速度、关闭碰撞或重力来制造通过。

**代码唤醒。** Mac 的 `physical_repair_cc_20261114/monitor_tools/`、`arm_round53_monitor.py`、`native_monitor/round53_config.json`、`native_monitor/round53_state/events/` 保存观察代码、轮次配置与实际回执。主要脚本和配置已核校备份至 `$H/mac_monitor/`；桌面投递服务在 Mac 运行。新聊天若获准运行，重新绑定聊天、轮号、源码、输出目录、实际 PID 和启动时钟，并核直接唤醒回执。短诊断直接守到结果，长仿真由代码静默监控。

## 空间清理与恢复

- 统一恢复入口是 **`$B/batch36_40_20261111/resource_cleanup/current_backup_access.json`**。其中 `after53_20261118` 指向本次逐文件收据。第44、50、51轮各3份大日志已经 gzip 无损保存，完整解压的大小、SHA-256 均与原件相同后才移除明文。历史读取器需要明文时，按收据取 `gzip` 路径，执行 `gzip -dc "$archive" > "$scratch_file"`，再用 `sha256sum "$scratch_file"` 对照收据。为旧读取器准备独立回放目录。[本次服务器清理收据](</Users/zhaoheng/Desktop/PCL work/Beam修复/physical_repair_cc_20261114/handoff_after53/server_cleanup_receipt.json>)
- 8个已关闭轮次中，24张重复结束帧与同轮编号 PNG 的 SHA-256 相同，原来的两个路径现在共享同一份文件数据；编辑派生图片前先复制。第52、53轮的诊断日志仍保持原格式。原始视频、独有图片、源码和回退件均保留。本机仅删除43份与现存源码时间及大小匹配的可再生成 Python 字节码。[本机清理收据](</Users/zhaoheng/Desktop/PCL work/Beam修复/physical_repair_cc_20261114/handoff_after53/local_cache_receipt.json>)
- 本次服务器文件占用减少 **3.20 GiB**，实测可用空间 **27.13 → 30.33 GiB**。此前 `closed_output_cleanup_20261118/` 是另一批收据。更早的大型证据有一部分迁往 `lab-40:/home/zhaoheng/beam_repair_evidence_backup_20261111`；最新访问方式和各组恢复步骤见同一 `current_backup_access.json`。历史路径是从 Mac 经 `pcl-server` 跳转至 .40，此次未复验 .40 连通性。旧轮曾明确清除的非关键中间 PNG 无法通过恢复索引重建。Mac 的 `physical_repair_20261110/closed_raw_backups_20261112/` 等恢复材料保留。

本机工作根为 `/Users/zhaoheng/Desktop/PCL work/Beam修复`。从这里可以打开[第53轮原件核验摘要](</Users/zhaoheng/Desktop/PCL work/Beam修复/physical_repair_cc_20261114/v230_cap_close/original53_review.json>)、[账本关闭收据](</Users/zhaoheng/Desktop/PCL work/Beam修复/physical_repair_cc_20261114/v230_cap_close/closure_receipt.json>)和[实现与检查脚本](</Users/zhaoheng/Desktop/PCL work/Beam修复/physical_repair_cc_20261114/v230_cap_close/>)。服务器补充入口为 `$H`，收集到的脚本及技术记录逐文件 SHA 见 `$H/files_sha256.json`。


## 历史机制与证据定位

查早期路线的排除项、指垫接触与承载判定，从[纯物理发现](beam_routes/physical/FINDINGS.md)进入。共用检查和[路线检查](beam_routes/physical/CHECKLIST.md)可帮助核验真实双臂、持件、扫掠空间和失败前后的指令。中间阶段的接触、局部成功或进程退出码不能代替五件最终验收。历史流程演变、旧保护阈值和逐轮完整文字可在服务器归档中按成员读取：

```bash
tar -xOzf "$H/docs_before_consolidation.tar.gz" R/BEAM_PHYSICAL_HANDOFF.md | less
tar -xOzf "$H/docs_before_consolidation.tar.gz" B/BEAM_FINDINGS_INDEX.md | less
```
