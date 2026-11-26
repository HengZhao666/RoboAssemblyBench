# Beam 纯物理 · 仿真耗时与日志体积诊断（2026-11-18）

**范围**　诊断以第53轮（v230）原输出、冻结启动包和生产源码为依据，对照 Plumbers C01（R06）。同日按人类指令“帮我进行对照测试”做了 §5 的短程对照：每次从初始场景跑到第 11,300 步后停止。随后按 “Profile it” 和 “Yes, go ahead” 做了 §7 的函数级 profile 和 §6 的运行时补丁对照，均跑到第 18,000 步。补丁只在对照包的 worker 层。生产源码、冻结第53轮运行包和账本均未改动，正式轮次号未增加。除标明“估计”的数字外，其余均来自原件或对照实测。

## 1. 结论

- 第53轮墙钟 **15,733 s（4 h 22 min）**，共 68,988 步，平均 **228 ms/步**。Plumbers C01 为 7,783 s / 69,051 步，**113 ms/步**。两轮的模拟时长都约 287 s。Beam 在第5件提起时失败，按剩余步数估计，完整一轮约 85k 步、5.4 h。
- 第53轮输出占 **6.3 GB**，运行中原始 JSONL 约 **18 GB**（gzip 前）。Plumbers 一轮 **157 MB**。
- **对照实测：日志是空间问题，不是主要的时间问题。** 在前 11,300 步，保持物理逐数相同的 compat 配置只快 5%（150 → 143 ms/步），输出从 416 MB 降到 121 MB。完全不加观察器的 lean 配置快 20%（120 ms/步），但轨迹改变。此前“接近 Plumbers、约快2倍”的估计**不成立**。
- **新发现：Beam 有几个“诊断”会改变物理。** 接触 trace 加上支撑接触 trace、grasp-sweep 补丁、被动实体几何监控，都会懒创建 `RigidPrim` 接触探针或视图。探针创建时 Isaac 会对被测刚体施加 `PhysxRigidBodyAPI`，并把 `sleepThreshold` 设为 0。是否开这些开关、在哪一步开，会改变持件闭爪的完成步和桌上自由零件的微小运动。第1–53轮的历史轨迹都是在这些探针存在的条件下得到的。
- 每步耗时随双臂作业上升：第53轮 `env_step` 从约 82 ms 涨到约 160 ms，`policy_act` 从约 37 ms 涨到约 100 ms。
- **函数级 profile（§7）：慢在 Python 状态读取，不在物理。** 重段 PhysX 物理加渲染只占 9–16%。`get_tracked_object_states` 每步调用2次，占 42–43%；`_physical_hold_valid` 每步约9次；`copy.deepcopy` 累计占约40%，反复深拷贝含采样点的接触度量。同一物理帧内的重复读取和深拷贝可以去掉，原则上不改变物理，但需逐数 A/B 验证。
- **运行时补丁实测（§6）：省 38%，物理逐数相同。** 跑到第 18,000 步，compat 加三项补丁后墙钟 3,055 → 1,885 s（170 → 105 ms/步）。阶段转移、进度行、五件与双臂终态、状态写入都与 compat 逐数相同。这三项是：只建探针不建记录、深拷贝改 pickle 往返、接触度量同帧缓存。按同比例估计，第53轮的 4.4 h 可降到约 2.6 h（Plumbers 为 2.2 h）。这只是估计：第 18,000 步以后的等价性和全流程耗时都还没跑。

## 2. 时间去向（第53轮实测）

| 区间 | 秒 | 占比 | 说明 |
|---|---:|---:|---|
| `env_step` | 8,761 | 55.7% | 物理（CPU TGS，五件 SDF 分辨率500）、任务更新、三相机每8步渲染、多数 trace（`phase_timing.json` 未细分） |
| `policy_act` | 5,965 | 37.9% | 控制器、IK、各 trace；关节 CAD 保护 375 s、闭爪 CAD 保护 481 s |
| 两者之外 | ≈1,007 | 6.4% | 被动碰撞监控 645 s，阶段轨迹预检约 100 s，视频与进度 |
| 视频编码线程 | 1,231 | 并行 | 三路各 8,773 帧 |

| 第53轮步段 | env_step ms/步 | policy_act ms/步 |
|---|---:|---:|
| 0–6,579（底梁抓取、搬运） | 82 | 37 |
| 6,579–14,367 | 98 | 51 |
| 14,367–23,421 | 158 | 103 |
| 23,421–48,412 | 123–168 | 80–116 |
| 48,412–68,153 | 100–158 | 76–104 |

## 3. 日志体积（第53轮）

| 文件 | 体积 | 来源 | 对物理的影响（§5 实测） |
|---|---:|---|---|
| `contact_trace.jsonl`（gz 前） | 14.44 GB | `BEAM_CONTACT_TRACE_PATH` + `BEAM_SUPPORT_CONTACT_TRACE` | **两者同开时，支撑接触采样会创建探针，改变物理**；写盘部分可丢弃 |
| `physics_boundary_trace.jsonl`（gz 前） | 3.35 GB | `BEAM_PHYSICS_DIAGNOSTICS*` | 无 |
| `grasp_sweep_trace.jsonl` | 656 MB | 第24轮 worker 补丁 | 关闭后桌上自由零件有微小差异；保留补丁、丢弃写盘时与原配置逐数相同 |
| `hold` / `table_support` / `assembly_integrity` / `ik` / `loaded_force` 等 | 约 1 GB | `BEAM_MOTION_TRACE_PATH`、`BEAM_HOLD_TRACE_PATH`、`BEAM_CLOSE_*` 等 | 无 |
| `entity_geometry_trace.jsonl` + 结果 JSON 中 197 MB 预检 | 515 MB + 2×197 MB | `BEAM_ENTITY_GEOMETRY_CHECK`、`--runtime-constraint-monitor`、`--stage-trajectory-precheck` | 关闭后桌上自由零件有微小差异 |
| 关键帧、留存帧、视频 | 308 + 151 + 78 MB | stride 8 / 30 fps，`--keep-video-frames`、`BEAM_VIDEO_CLOCK_TRACE` | 无 |

被动监控记录的 4,913 次“违规”主要是腕部胶囊近似造成的误报，不会终止任务。两个 CAD 保护会终止任务，必须保留。

## 4. 建议

1. **马上可用，物理逐数相同：compat 配置。** 保留原 worker 链和所有会创建探针或视图的观察器，把它们的大文件指向 `/dev/null`。关闭物理诊断、motion/hold/close 等 trace、视频时钟 trace 和逐帧留存，视频改为 stride 24 / 10 fps。实测前 11,300 步与原配置逐数相同，输出降到 29%，墙钟快约 5%。入口：`perf_ab_20261118/run_ab.sh original` 加 `BEAM_AB_COMPAT=1`。需要排查失败时，用原配置或停止帧回放补开诊断，物理不变。
2. **先消除观察器对物理的影响，再谈真正的精简（需改生产源码，要单独授权）。** 在场景复位后，对所有可能被探针测量的刚体统一预先施加同样的 contact-report/sleep 设置；或改为只读已有视图，探针不再修改刚体属性。之后观察器开关就不再影响轨迹，lean 配置（快 20%）和全量诊断可以互换。改动后的轨迹与第1–53轮不再逐数可比，需要一次全流程基线。
3. **按 profile 去掉重复计算（改生产源码，需授权；目标与原配置逐数相同）。** 已用 cProfile 定位，见 §7。按预期收益排序：
   - (a) `get_tracked_object_states` 按物理帧缓存。policy 与 obs 在同一帧各调用一次，第二次纯属重复，约省 20%。
   - (b) 深拷贝 attachment 时不复制 `sample_contacts` 等大数组。只在确有修改的地方复制，或保存精简快照。
   - (c) `_gripper_contact_metrics` / `_physical_hold_valid` 按（物体、物理帧、attach_spec）缓存，从每步约9次降到1–2次。`_physical_hold_valid` 的副作用只有样本号、最近一次结果和 hold trace，需确认这些诊断计数的口径变化可以接受。
   - (d) compat 下，接触 trace 的记录构建（重段 21%）改为只按原时序创建同一组探针、不构建记录。这一项可以在对照包的 worker 层实现，不改生产源码。

   Plumbers 上游已有同类实现：`physics_read_cache.py`、`physics_frame_cache.py` 和 state read cache。每项改完都用 `run_ab.sh` 与原配置做逐数 A/B。CAD 保护在 profile 中约为 0，不需要优化。

   **实施情况（§6）**：(b)(c)(d) 已作为对照包的运行时补丁实现，并实测到第 18,000 步。(b) 用 pickle 往返替代深拷贝，不删字段。(c) 只缓存纯读取的 `_gripper_contact_metrics`，`_physical_hold_valid` 仍逐次执行，口径不变。(a) 尚未实现，是下一个最大的单项。移入生产源码需单独授权。
4. **任务执行速度（改变物理过程，需另行授权）。** 步数主要花在 `assembly_clearance`（4次共 17,268 步）和 `descend`（8次共 10,963 步）。持件段限速 0.04–0.06 m/s。当前失败就是持件丢失，所以只建议先试空臂段（约 9.5k 步）。物理步长、SDF 分辨率和求解器迭代不建议改动。

## 5. 对照测试结果（2026-11-18）

所有配置使用同一 v230 源码、worker 配置、seed 0 / layout 505、TGS/CPU，900 文件 SHA 在运行前后均相符。到第 11,300 步时一律以 `ab-step-limit` 失败结束，失败尾录关闭（`BEAM_POST_FAILURE_VIDEO_SECONDS=0`）。各配置单独运行时测墙钟。

| 配置 | ms/步 | 墙钟 | 输出 | 与原配置比较 |
|---|---:|---:|---:|---|
| original：第53轮的诊断、监控、视频设置 | 150.3 | 1,699 s | 416 MB | 19 次阶段转移与第53轮原件逐步相同 |
| **compat**：原 worker 链，保留探针类观察器，丢弃字节 | **142.8** | 1,614 s | **121 MB** | **阶段转移、五件与机械臂终态、状态写入全部逐数相同** |
| lean + contact→/dev/null | 129.1 | 1,459 s | 50 MB | 阶段转移、机械臂、持件 beam_6 逐数相同；桌上未动的 beam_0–3 终态差 ≤0.73 mm / 9.2 mrad |
| lean：无 trace、无被动监控、无 sweep/shell 补丁 | 120.3 | 1,360 s | 53 MB | 从 `base_6_close_and_attach` 起不同：闭爪提前 44 步（1,926 对 1,970），之后整体前移 |

逐组分离结果（lean 加一组开关）：
- 只有 `contact`（接触 trace 路径加支撑接触同开）能恢复第 1,970 步；单开其中任一项都不能。
- `video`、`othertraces`、`physdiag` 对物理无影响，加上后的结果与 lean + contactnull 完全一致。
- `monitors` 使自由件差异由 0.73 mm 降到 0.40 mm，但不归零。其余差异来自原 worker 链的 sweep/shell 补丁。

服务器目录 `grasp_repairs_20261012_20rounds/perf_ab_20261118/`（约 2.7 GB）：
- 入口与工具：`ab_entry.py`、`lean_worker.py`、`opt_worker.py`、`run_ab.sh`、`compare.py`、`timing_chain.sh`
- 授权：`authorization.json`
- 各配置输出：`original/`、`orig_compat/`、`orig_compat_profile/`、`lean*/`、`bis_*/`、`opt_*/`、`timing_*/`
- 比较结果：`comparison*.json`。用法：`compare.py <配置> [参照]`

本机副本在 `physical_repair_cc_20261114/perf_ab_20261118/`（含 `results/`）。compat 已确认到第 18,000 步仍与第53轮原件转移相同，见 §6、§7。全流程的等价性和时间收益仍待验证。

## 6. 运行时优化补丁对照（2026-11-18）

按人类指令 “Yes, go ahead”，在对照包的 `opt_worker.py` 中实现 §4 第3条的 (b)(c)(d)。三项都是运行时补丁，由环境变量逐项开关，生产源码未改。

`opt` 模式的观察器与 compat 相同：接触 trace 和实体几何 trace 指向 `/dev/null`，支撑接触、被动监控和预检照开，视频 stride 24 / 10 fps。worker 层是 `lean_worker.py`，保留全部 CAD 保护、状态写入审计、阶段计时、进度和资源停止。

| 开关 | 做法 | 对应 §4 |
|---|---|---|
| `BEAM_OPT_PROBE_TRACE=1` | 替换 `_write_physical_contact_trace`。按原去重键、步号门限和顺序，只调用 `_get_contact_probe` 创建同一组探针，覆盖支撑接触段、pregrasp 邻件段和第24轮 sweep 段。不构建、不写记录 | (d) |
| `BEAM_OPT_FASTCOPY=1` | 任务模块、`beam_support_state`、`beam_contact_relief` 中的 `copy.deepcopy` 改为 pickle 往返，保留数值和共享引用。无法 pickle 时退回 `copy.deepcopy` | (b) 的替代做法 |
| `BEAM_OPT_FRAME_CACHE=1` | 纯读取的 `_gripper_contact_metrics` 按物理帧缓存（`SimulationContext` 的步号和时间），键为（物体、机器人、attach_spec）。力探针无效的结果不缓存；每次任务状态写入（`_trace_beam_state_write`）都清空缓存 | (c) 的保守版 |

结果：两次单独运行，同一机器，都从初始场景跑到第 18,000 步，以 `ab-step-limit` 停止。

| 配置 | 墙钟 | ms/步 | `policy_act` | `env_step` | 输出 | 比较 |
|---|---:|---:|---:|---:|---:|---|
| `timing_compat` | 3,055 s | 169.7 | 977 s | 1,764 s | 225 MB | 与带 profile 的 compat 逐数相同；29 次转移与第53轮相同 |
| **`timing_opt_all`**（三项全开） | **1,885 s** | **104.7** | **311 s** | **1,251 s** | 224 MB | **与 compat 的阶段转移、进度行、五件与双臂终态、状态写入全部逐数相同** |

- 墙钟省 38%，相当于 1.62 倍速。`policy_act` 降 68%，`env_step` 降 29%。
- 持件段降幅最大：`assemble_00_part_3_pickup_clearance` 408 → 170 s，`assemble_00_part_3_lift` 221 → 90 s。空臂段基本不变，例如 `base_6_move_above` 59 → 58 s。
- 补丁计数：fastcopy 373,993 次，回退 0。帧缓存命中 96,805 次、未命中 20,757 次（命中率 82%），不可缓存 0，状态写入清空 17 次。探针 trace 39,400 次，sweep 849 次。
- 三种组合（`opt_probe`、`opt_probe_copy`、`opt_all`）另各跑到第 18,000 步，都与带 profile 的 compat 逐数相同。这三次同时运行、墙钟互相干扰，只用作等价性证据，不拆分单项收益。
- `opt_probe` 与 compat 相同还说明：至少在前 18,000 步，原 worker 链对物理的影响只来自探针创建，第24轮的 shell trace 不影响物理。
- 输出大小不变。225 MB 主要是视频和保留的监控结果，这些补丁只省时间。

入口：`BEAM_AB_VARIANT=<名> BEAM_AB_STOP_STEP=<步> BEAM_OPT_PROBE_TRACE=1 BEAM_OPT_FASTCOPY=1 BEAM_OPT_FRAME_CACHE=1 bash run_ab.sh opt`。顺序计时用 `timing_chain.sh`，统计写在各输出的 `opt_stats.json`。本机副本在 `results/comparison_timing_*.json`、`results/opt/`。

仍未验证：
- 第 18,000 步以后是否仍逐数相同。这包括第3件之后的插装、关节技能段和闭爪 CAD 保护介入段。
- 全流程实际耗时。

下一步可跑 compat 与 opt 的全流程对照（不改生产源码）。之后再决定是否把补丁和 (a) 移入生产源码，这需要单独授权；移入后照此用 `run_ab.sh` 复核。

**正式使用（2026-11-19 起）。** 第54轮起，正式轮次使用这套优化 worker：`round54_v231_cap_depth/tools/worker.py` 加 `tools/lean_worker.py`。

- 唯一新增：失败快照，以及第 65,000 步以后每 8 步一条的完整接触记录，改为调用原写入函数。原函数建立的探针与只建探针路径相同。
- 两次冒烟（420 步、2,600 步；后者从第 1,800 步起开启完整 trace）与 `timing_opt_all` 的进度逐行相同。
- 第54轮前 65,5xx 步与第53轮代码和配置相同，用 `prefix_check.py` 对照两轮的进度行，可检验全流程前缀是否等价。

## 7. 函数级 profile（2026-11-18，compat 配置）

按人类指令 “Profile it”，用 Python 自带的 cProfile（不下载工具）对 compat 配置的主线程采样。运行从初始场景跑到第 18,000 步，采两个窗口：第 2,000–5,000 步（底梁提起与搬运，轻段），第 14,400–18,000 步（第3件抓取与提起，重段）。本次 29 次阶段转移与第53轮原件逐步相同，所以 compat 至少到第 18,000 步仍与原配置等价。cProfile 会放大调用密集的 Python 函数：重段采样 1,313 s，同段无采样约 940 s。下表比例偏向 Python，物理的实际占比略高。

| 累计时间（含子调用） | 轻段 586 s | 重段 1,313 s | 说明 |
|---|---:|---:|---|
| `world.step`（PhysX `fetch_results` + 渲染） | 95 s（16%） | 123 s（9%） | 真正的物理很便宜 |
| `get_tracked_object_states` | 249 s（42%） | 560 s（43%） | 每步2次：policy 内 143/308 s，obs 内 106/252 s；每次深拷贝整个 attachment_state 并重算持件有效性 |
| `_physical_hold_valid` | 197 s | 646 s | 重段 33,820 次，约 9.4 次/步：来自 tracked states、`_sync_object_states` 和控制器 |
| `copy.deepcopy` | 183 s | 532 s | 重段调用 3.8 亿次，主要来自上面两项和接触 trace |
| `_gripper_contact_metrics` | 96 s | 210 s | 含 123 万次采样点坐标变换 |
| compat 接触 trace 记录构建（写 /dev/null） | 32 s | 282 s | 只为保持探针时序；记录本身被丢弃 |
| 控制器 `action_for` | 67 s | 224 s | 含 `_hold_previous_grasp_command`、四元数运算 |
| 被动碰撞监控 `observe` | 32 s | 40 s | |
| 关节 / 闭爪 CAD 保护、视频 | ≈0 / 2 s | ≈0 / 2 s | 这两段没有进入关节技能或改动的闭爪段 |

结论：Beam 比 Plumbers 慢，主要因为 Python 侧的同帧重复读取和深拷贝，与日志量、物理设置关系不大。按 §4 第3条的 (a)–(d) 处理，估计每步可省 40–60%。这是根据 profile 比例的估计。§6 实测其中 (b)(c)(d) 三项省 38%；(a) 尚未实现。

文件：服务器 `perf_ab_20261118/orig_compat_profile/profile_{2000_5000,14400_18000}.{pstats,cumulative.txt,tottime.txt,json}`；本机 `physical_repair_cc_20261114/perf_ab_20261118/results/profile/`。采样开关为 `ab_entry.py` 的 `BEAM_AB_PROFILE_WINDOWS`。

## 7b. 生产链 profile 与速度度量同帧缓存（2026-11-19，第56轮前）

按人类指令"如果还能加快仿真速度 请加快"，用第55轮的生产启动链（v232 配置、优化 worker、全部观察器）跑到第 4,700 步，对第 2,000–4,600 步（底座提起与搬运）做 cProfile。入口 `perf_r56_20261119/perf_run.sh` 与 `profile_entry.py`，结果在 `perf_r56_20261119/profile_v232/profile_2000_4600.*`。

- 约 107 ms/步（含 profiler 开销）。PhysX `fetch_results` 约 29 ms/步，占约 28%；渲染每 24 步一次，开销很小。
- `_object_velocity_metrics` 每步约 24 次，累计 62 s（约 22%）。其中 152 万次 `pose_error` 用于在 24 帧位姿历史上算漂移。它是纯读取：结果只取决于当前帧的刚体速度、该物体的位姿历史和调用参数。
- 其余较大项：`get_tracked_object_states` 58 s（每步 2 次，含 `_physical_hold_valid` 的诊断副作用，未缓存）；物体状态健全检查 43 s；被动碰撞监控 28 s（会创建视图，保留）。

**实施：`BEAM_OPT_VELOCITY_CACHE=1`（`round56_v233_fast/tools/worker.py`）。** 按物理帧、物体、关键字参数和位姿历史指纹（身份、长度、首尾样本的精确值）缓存 `_object_velocity_metrics`；任何任务状态写入都会清空缓存，命中时返回独立副本。A/B（`perf_r56_20261119/vc_v232`，同一链路加此补丁）跑到第 4,700 步：
- 28 行进度，以及 hold（14,624 行）、motion（2,728 行）、attach（672 行）、close-gate（743 行）trace，与第55轮原件逐行相同。
- 第 2,000–4,600 步 90.4 → 82.1 ms/步，约快 10%。命中 56,485 次、未命中 56,471 次；同一帧内位姿历史更新前后的调用键不同，所以命中率约一半。
- 结果文件 `offline_v233_fast/velocity_cache_ab.json`。第56轮起正式使用。

未做：`get_tracked_object_states` 的第二次调用缓存。这次调用有诊断副作用（hold 样本号、最近结果），在速度度量已缓存后只剩约 5 ms/步，收益小于风险。

## 8. 证据位置（服务器）

- 第53轮原件在 `$O` = `candidate_repository/outputs/fabrica_beam_v230_cap_close`：`phase_timing.json`、`capture_performance.json`、`trace_storage_manifest.json`、`joint_cad_guard.json`、`close_cad_guard.json`、`episode_0000.json`。
- 启动链：`$Q/run_full_assembly.sh`、`$Q/tools/worker.py`、`$B/round24_native_release/full_assembly_worker.py`。
- 探针副作用的代码位置：`internutopia_extension/tasks/factory_dual_franka_assembly_task.py` 的 `_get_contact_probe`、`_write_physical_contact_trace` 支撑接触段；Isaac `isaacsim.core.prims/impl/rigid_prim.py` 的 `_apply_rigid_body_apis`（`sleepThreshold=0`）。
- Plumbers 对照：`/home/user/work/zhaoheng/RoboAssemblyBench-integrated/outputs/plumbers_reproduction/20261114_033615_repair_C01_R06_handoff_release/native/{validation_config.json,rollout_performance.json}`。
