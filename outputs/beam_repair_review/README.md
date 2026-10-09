# Beam 修复：关键实验视频和日志

这些是同种子修复过程中选出的四个审查节点，不是成功演示合集。每个节点均保留停止原因；所有节点的完整装配结果均为失败。环境为服务器 Isaac Sim 5.1.0，seed=0，layout_seed=505。

## v138：去掉抓持登记的位姿/速度副作用

检查“抓住的登记”是否改变自由零件运动。通过登记后，底座在 grip_settle 阶段丢失一侧接触，被真实接触保护停止；不能称为成功搬运。

- [正面视频](v138/videos/observation_images_front.mp4)、[右腕视频](v138/videos/observation_images_right_wrist.mp4)
- [运行结果](v138/collect_results.json)、[登记前后和失夹分析](v138/registration_analysis.json)
- [接触失败原始记录节选](v138/contact_failure_excerpt.jsonl)、[运行日志](v138/runner.log)

## v151：抓姿与运动连续性改善后的搬运节点

运行完成底座提取和搬运，并按当时的阶段门槛进入首根立柱取件；之后在立柱 close_and_attach 阶段出现 contact_pose_gate_lost。当前落桌验收更严格，**该历史运行不能证明通过当前的落桌门槛**。

- [正面视频](v151/videos/observation_images_front.mp4)、[右腕视频](v151/videos/observation_images_right_wrist.mp4)
- [阶段和动作统计](v151/round_summary.json)、[完整结果元数据](v151/collect_results.json)
- [抓取碰撞检查](v151/pick_collision_audit.json)、[运行日志](v151/runner.log)

## v164：固定空闲臂目标，底座接触承重改善

恢复 TGS，保持原机械臂增益，只增加 Beam 空闲臂固定原生命令目标。末 2 秒桌面平均承重约 2.391 N，约占底座重量 97.5%；原始角速度平均约 0.376 rad/s，位姿差分约 0.0135 rad/s。仍在 base_6_set_down 超时，严格稳定窗口为 0，没有插入和换扶运行。

- [正面视频](v164/videos/observation_images_front.mp4)、[右腕视频](v164/videos/observation_images_right_wrist.mp4)
- [结果与承重统计](v164/round_summary.json)、[空闲目标保持分析](v164/idle_hold_summary.json)
- [四边界分析](v164/boundary_analysis_steps_11880_12480.json)、[边界原始日志节选](v164/physics_boundary_excerpt.jsonl)
- [空闲臂原始记录](v164/idle_hold_trace.jsonl)、[该次生成配置](v164/compiled_recipe.json)、[运行日志](v164/run.log)

边界节选选择任务周期 11900～11920，保留四个采样点。部分指垫缓存缺失会使总 valid 标记为 false；分析使用有效的底座、夹爪本体和关节字段，不把缺失字段当作可信数据。

## v166：较低增益试验暴露下放保护卡住，参数已回退

试验 kp40000/kd2000。底座到桌面仍有约 2.92 mm 间隙，4800 个落桌采样承重均为 0；下放请求被 pose_tracking_pause 拦住 4776 次。完整装配失败，较低增益没有保留。它展示的是下一步要定位的控制保护问题。

- [正面视频](v166/videos/observation_images_front.mp4)、[右腕视频](v166/videos/observation_images_right_wrist.mp4)
- [运行结果](v166/round_summary.json)、[驱动试验分析](v166/gain_trial_summary.json)
- [下放保护原因](v166/seating_guard_summary.json)、[该次生成配置](v166/compiled_recipe.json)、[运行日志](v166/run.log)

## 配套诊断、当前保留状态及证据完整性

- [v161 零件与夹爪位姿/关节速度比较](v161_diagnostics/gripper_pose_consistency_diagnosis.json)：这一时间段可见小幅相对运动，关节速度为每 8 步采样，不能完全排除混叠。没有完整夹爪六维速度，不能直接与零件速度逐项比较。
- [v162～v166 汇总](batch_162_166/review.json)、[每轮量化结果](batch_162_166/completed_round_metrics.json)
- [最终恢复参数与验证](final_retained/final_verification.json)、[最终源码清单](final_retained/final_source_manifest.json)、[历史 201 项检查日志](final_retained/tests.log)
- [文件来源、原始与公开 SHA-256](provenance.json)

视频直接保留原文件，不重新剪辑或重编码；共 8 个视频。文本仅替换个人/机器绝对路径，文件来源和原始哈希可查。两个 JSONL 节选保留选中窗口的完整数值记录，其他大型逐步日志没有全部加入 Git。旧实验的源码清单可能只覆盖当轮修改文件，不应误认为完整运行清单。

**当前代码已经恢复 kp80000/kd4000，未保留 PGS。恢复后做了源码、配置和离线检查，没有重新运行完整仿真。**
