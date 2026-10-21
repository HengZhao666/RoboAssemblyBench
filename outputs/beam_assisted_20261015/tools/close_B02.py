"""Close existing B02 evidence and coordination state; never starts simulation."""
from pathlib import Path
import json, hashlib, shutil

L = Path(__file__).resolve().parents[3]
P = L / 'outputs/beam_assisted_20261015'
D = P / 'A12_evidence'
def read(p): return json.loads(p.read_text())
def write(p, v): p.write_text(json.dumps(v, ensure_ascii=False, indent=2) + '\n')

result = read(D / 'collect_results.json')[0]
source = read(D / 'source_after.json')
assert result['failed'] and not result['success'] and result['steps'] == 18273
assert not source['differences'] and not source['tool_differences']
assert read(P / 'A12_video_download_verification.json')['all_equal']
assert read(P / 'A12_key_nodes_download_verification.json')['all_equal']

indices = [904, 913, 918, 1483, 1555, 2283, 2284, 2433]
frames = {}
for line in (D / 'video_frame_clock.jsonl').read_text().splitlines():
    r = json.loads(line)
    if r['frame_index'] in indices:
        frames.setdefault(str(r['frame_index']), []).append(r)
for rows in frames.values():
    assert len(rows) == 3
    assert len({r['physics_stamp']['index'] for r in rows}) == 1
visual = {
    'scope': 'Root inspected original three-view montages, not every video frame',
    'frames': frames,
    'observations': {
        '904_913_918': 'Base opening sequence shows full housing, opening jaws and separated base; no obvious gross interpenetration in inspected images',
        '1483_1555': 'Part3 appears closely held between both pads during grip and lift; full housing visible',
        '2283_2284_2433': 'Insertion stop and tail preserve close grasp and housing. Micrometer CAD intersection cannot be certified absent from RGB; native/CAD failure remains authoritative',
    },
    'earlier_reviewed_frames': [311, 968, 1819],
    'limits': ['Sampled visual inspection only', 'No full-sequence visual perfection claim', 'No tail CAD depth stream'],
}
write(P / 'B02_visual_keyframe_review.json', visual)

summary = {
    'cycle': 'B02', 'run': 'A12_beam_full', 'status': 'closed_failed_then_paused_by_user',
    'source': 'assisted_v16b', 'source_files': len(source['files']),
    'critical_tools': len(source['tool_files']),
    'source_differences': source['differences'], 'tool_differences': source['tool_differences'],
    'task_steps': result['steps'], 'success': False, 'assisted_success': False, 'formal_success': False,
    'terminal_reason': 'assisted-external-evidence-stop',
    'first_failure_task_step': 18272, 'first_failure_physics_index': 18275,
    'first_failure_phase': 'assemble_00_part_3_insert_07',
    'actual_new_progress': ['native/USD startup publication and actual three-image gate passed',
        'base6 assisted grasp/lift/transport/correct placement/full opening/release/retreat/park passed',
        'part3 assisted grasp/lift/transport/hover and insert00-06 passed'],
    'failure': {'pair': ['fabrica_beam_6/mesh', 'fabrica_beam_3/mesh'],
        'visible_triangle_contact_parameter_m': 3.317869259594937e-6,
        'existing_numerical_threshold_m': 1e-6, 'actual_xy_error_m': .001043946,
        'remaining_height_above_insert07_target_m': .008666221,
        'pair_force_on_part3_N': [.000193, -3.679965, 2.333017],
        'pair_force_norm_N': 4.35719,
        'xy_only_frozen_pose_correction_clearance_m': .000814381,
        'interpretation': 'Actual lateral pose error is relevant to the geometric collision. Nominal preflight did not cover the permitted/actual error envelope. Unique dynamical cause is unproven. FCL parameter is not native cooked penetration depth.',
        'command_handoff': 'No old-target jump found in reviewed failure window; published/SDK/native targets agree within floating-point tolerance'},
    'unreached': ['part3 completed installation/release', 'remaining parts2/1/0 native sequence', 'five-part final common240 window'],
    'base_release': read(P / 'B02_base_release_independent_review.json'),
    'native': {'samples': 9140, 'valid_samples': 9140, 'failure_tail_physics_ticks': 1200,
        'failure_tail_seconds': 5.0000002608, 'tail_raw_records_including_entry': 151,
        'tail_additional_valid_samples': 150, 'new_policy_actions_in_tail': False,
        'tail_part3_displacement_down_m': .00141954,
        'tail_scope': 'Holds last published targets; observation only, not successful insertion or release'},
    'preflight': {'checks_passed': 330, 'startup_contract_checks': 6,
        'actual_layouts': 4, 'nominal_visible_samples': 7423, 'nominal_camera_checks': 16972,
        'actual_dynamics_or_error_envelope_certified': False},
    'video': {'originals': 3, 'frames_each': 2434, 'duration_s_each': 81.133333,
        'bytes_total': 25433249, 'key_clips': 5, 'full_decode_passed_on_server': True,
        'local_server_size_sha_equal': True, 'three_view_physics_alignment_verified': True,
        'local_original_directory': str((D / 'episode_0000_live_videos').resolve()),
        'local_key_clips': str((P / 'A12_key_nodes').resolve())},
    'full_raw_archive': {'server_path': '/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015/A12_beam_full_evidence.tar.gz',
        'bytes': 117986229, 'sha256': '39409905fdef6eb74834f636d0407d02aa4186a485b4b79cdd5129c4f00c043c',
        'downloaded': False, 'original_pngs': 'Preserved in original server run directory'},
    'review_bundle': {'bytes': 33286044, 'sha256': '653e10458e87656957ab27cfcfeb5500ef501ecfc029d54b497171b1c64ae469', 'downloaded_and_verified': True},
    'independent_reports': ['B02_failure_target_review.json', 'B02_failure_control_review.json',
        'B02_camera_review.json', 'B02_visual_keyframe_review.json', 'B02_mesh_coverage_review.json'],
    'coverage_limits': ['Nominal exact paths do not certify actual tracking/error envelope',
        'Mesh sampling does not certify between-sample CCD, unexported warehouse or cooked/SDF behavior',
        'Tail lacks CAD entity trace', 'Selected native pairs are not a complete force balance',
        'RTX render-product matrix binding checked at startup; rollout matrix audit reads USD',
        'Assisted fixed joints/world support do not prove pure contact physical assembly'],
    'retained': ['v201 physical876 source and original rollback', 'v16b887 source and9 critical tools unchanged', 'all previous versions/logs/rejected evidence'],
    'next_step_if_user_resumes': 'Review actual XY/orientation error envelope and lateral admission before near-interface descent; prove adequate CAD clearance on that envelope, then source-bound offline and native validation. Do not simply increase collision epsilon or tune force first.',
    'next_native_run_authorized': False, 'B03_B05': 'deferred_by_latest_user_instruction',
    'runtime_edits_after_failure': False, 'new_simulation_after_failure': False,
}
write(P / 'B02_postreview_conclusions.json', summary)

batch = read(P / 'review_batch_20261015.json')
batch.update(status='paused_by_user_after_B02', completed_cycles=2,
    next_native_run_authorized=False, current_run_ended=True,
    latest_user_instruction='这一轮结束后暂停。B02完整收尾后不启动B03，B03-B05延期。')
b = next(r for r in batch['cycles'] if r['id'] == 'B02')
b.update(state='closed_native_failure', native_finished_at='2026-10-15T16:54:07+08:00',
    task_steps=18273, success=False, assisted_success=False, formal_success=False,
    postreview='B02_postreview_conclusions.json', original_videos_download_verified=True,
    key_node_clips=5, source_after_differences=0, tool_after_differences=0,
    new_progress=summary['actual_new_progress'], failure=summary['failure'])
write(P / 'review_batch_20261015.json', batch)
ledger = read(P / 'round_ledger.json')
r = next(r for r in ledger['rounds'] if r['name'] == 'A12_beam_full')
r.update(state='closed_insert07_CAD_intersection_then_user_pause', steps=18273,
    source_before_after_differences=0, tool_before_after_differences=0,
    base_place_release_retreat_passed=True, part3_insert07_passed=False,
    native_samples=9140, native_valid_samples=9140, original_video_download_verified=True,
    key_node_clips=5, postreview='B02_postreview_conclusions.json')
ledger.update(status='paused_by_user_after_B02', pause_after_run='A12_beam_full',
    latest_user_instruction=batch['latest_user_instruction'], next_native_run_authorized=False,
    current_run_ended=True)
write(P / 'round_ledger.json', ledger)
contract = read(P / 'experiment_contract.json')
contract.update(status='B02_closed_failed_user_paused', current_run_ended=True,
    next_native_run_authorized=False, latest_user_instruction=batch['latest_user_instruction'],
    result={'steps': 18273, 'success': False, 'assisted_success': False,
        'base_release_retreat_passed': True, 'stop': 'part6-part3 visible CAD intersection at insert07',
        'postreview': 'B02_postreview_conclusions.json'},
    pending_fixes_not_implemented=['Actual near-interface lateral error envelope/admission review; user resume required'])
write(P / 'experiment_contract.json', contract)

history = P / 'coordination_history'
history.mkdir(exist_ok=True)
for name in ['BEAM_REPAIR_HANDOFF.md', 'BEAM_ASSISTED_HANDOFF.md', 'BEAM_FINDINGS_INDEX.md']:
    target = history / ('before_B02_closure_' + name)
    if not target.exists(): shutil.copy2(L / name, target)

main = (L / 'BEAM_REPAIR_HANDOFF.md').read_text()
start = main.index('更新：')
end = main.index('\n\n', start)
main = main[:start] + '更新：2026-10-15。最新用户要求“这一轮结束后暂停”已执行到B02收尾：A12/v16b原生18273步失败结束，底梁辅助放置、开夹、释放和退避新增通过；首柱insert07实际横向偏差约1.044mm，底梁6/柱3可见CAD相交3.318微米触发停止。完整五件/全程视觉仍未通过，formal_success=false。B01、B02两次闭环完成；此前五次计划的B03–B05延期，不启动新仿真或新修复。旧v201纯物理路线冻结。辅助详情见根`BEAM_ASSISTED_HANDOFF.md`；不自动公开/更新PR。' + main[end:]
marker = '## 辅助路线当前批次（2026-10-15恢复）'
main = main[:main.index(marker)] + '''## 辅助路线当前状态（2026-10-15，B02后暂停）

- 审查流程和辅助专属交接已建立；本批B01、B02闭环完成（2/5），不称五轮完成。最新用户暂停覆盖剩余连续迭代授权；不启动B03–B05、不修复后重跑。
- P=`R/beam_assisted_20261015`，A=`P/assisted_repository`为服务器辅助权威源码；本机A_L=`L/outputs/beam_assisted_20261015/assisted_repository`。A12结束于16:54:07+08:00，v16b/source887及关键工具9前后零差。旧C876与W运行源码冻结，W HEAD431050c不变。
- 选择入口：`L/roboassemblybench/scripts/run_beam_route.sh --route physical|assisted --run-name NAME [--max-steps N]`。两路线保留；当前都暂停，入口可选不等于授权启动。原v201完整保存包`P/preserved_v201_source876.tar.gz`，SHA`b29ec75309254201f7ceefbe9fa6d02ef1cf085917b864455e7990e134e052c3`。
- 三路A12完整原视频已下载到`L/outputs/beam_assisted_20261015/A12_evidence/episode_0000_live_videos`；各2434帧、完整解码通过、两端大小/SHA同。五个三视角同物理时刻短片在`A12_key_nodes/`，包含首柱失败及5秒尾录，不包装成安装成功。
- 本轮分析、限制、版本回退和恢复验收见辅助交接及`P/B02_postreview_conclusions.json`。五协调文档同步W/A_L及服务器R/C/旧基线/A；最终逐SHA、旧876/辅助887/工具9、进程/锁与证据同步核验见`P/B02_final_pause_certificate.json`。没有该证书时不得宣称同步完成。
'''
(L / 'BEAM_REPAIR_HANDOFF.md').write_text(main)

(L / 'BEAM_ASSISTED_HANDOFF.md').write_text('''# Beam 辅助路线交接

更新：2026-10-15。当前B02/A12收尾后暂停；不修改运行源码、不新开仿真、不启动B03。最新用户“这一轮结束后暂停”覆盖此前连续五轮授权。B01、B02闭环完成（2/5），B03–B05延期；不能写五轮完成或完整装配成功。

## 权威路径、两路线与版本

- 总入口为根`BEAM_REPAIR_HANDOFF.md`；服务器源码是权威，本机是同步副本。L=`/Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench`；R=`/home/user/beam_integration_20261010_upstream6f57dae`；P=`R/beam_assisted_20261015`；服务器辅助A=`P/assisted_repository`，本机A_L=`L/outputs/beam_assisted_20261015/assisted_repository`。SSH沿用pcl-server。
- 旧物理C=`R/candidate_repository`冻结v201/source876；W=`L/submission/beam_upstream_integration_2026-10-10`，HEAD431050cda779b9a31cae21c548c761705e3b9482有意未提交，不reset/clean/add-A。
- 每次选择`L/roboassemblybench/scripts/run_beam_route.sh --route physical|assisted --run-name NAME [--max-steps N]`，名称不覆盖；辅助recipe=fabrica_beam_ur5e_assisted。当前暂停，不自动运行任一路线，不公开或更新PR，不操作其他项目进程。
- 最新辅助v16b/source887、关键工具9；A12前后逐SHA零差。版本清单为`P/tools/assisted_manifest.json`和`assisted_tools_manifest.json`；旧物理用physical_manifest.json。最终当前复核见B02_final_pause_certificate.json。
- v16b源回退：`P/versions/assisted_v16b_before_A12_final.tar.gz`，SHA64d2b28f26264f188723f0a7bc56804028b287f00f5db3db2dac4a4c19c1fc42；工具同目录`assisted_v16b_tools_before_A12_final.tar.gz`，SHA8d807eb94bc979a3806e701ad0f49102664ad0f71226fdcfc75d3d600b7a0599。v16原型未运行，v1–v15b及旧876完整回退/原日志均保留。

## 审查流程和已合并修复

- 用户批准分层启动门、尽量合并独立修复、每3次正式尝试广审；阶段切换/跨模块/两轮同路线无实质改善立即广审。本批原五次要求每轮全面审查；执行根BEAM_REVIEW_CHECKLIST.md。0动作初始化失败如实计工程闭环，不能算装配进步。
- B01合并8项：随机化目标绑定、真实解释器兼容、全开后缓落完成及native12稳定、五件共同240资格、完整夹爪多点相机遮挡、真实相机路径/生命周期、源码绑定启动门、完整预算。296项检查通过；A11b因物理已初始化而USD仍旧姿态，在首policy前被真实三图拒绝，0动作。32body差617.358mm/2.094398rad。工程修复保留，B01没有新增装配阶段。
- B02全面审查后合并：SDK发布已有物理结果到USD并核状态不变；完整Pydantic子类/真实runtime输入签名；真实最终_check_success与终态同clock证书；所有insert目标截到assembled+3mm且禁止cached命令越路点。330检查、6启动合同mock、4真实随机布局通过；新名义7423完整实体状态、16972相机检查通过。原始报告和失败反例保留，不等同动态路径证明。
- A12真实startup：44body最大位置差617.357535mm降到0.000029802mm，角差约2.9951e-7rad；native q/body/clock/四计数/BaseTask.steps/history逐值不变，无额外simulate、无物体状态写入。真实RGB产品矩阵/路径及三原图通过。最终五件入口修复仍未原生到达。

## 允许的辅助与成功边界

- 实测双侧可见CAD表面5–250微米、无交叠、12独立native clock稳定后，按实测位姿建立持件fixed joint。原力门独立报告，不把CAD资格或固定关节写成纯物理夹稳。
- 完整碰撞保持；不写Beam位姿/速度、不清速度、不切kinematic。实际接近原随机化assembled目标上3mm后建立世界支撑；6夹爪实际全开≤.05rad后，每物理tick锚点≤.1mm/.2mrad缓落。原捕获和完成门未放宽。辅助世界支撑不等于自然承重/落桌。
- 辅助最终要求五件全释放、原目标3mm/.04rad、共同240物理步稳定及全序列完成；formal_success始终false。本轮assisted_success=false。视觉紧贴/完整壳体/无相交必须另验，本轮不能宣称视觉毫无问题。

## B02/A12实测进展与失败

- A12于16:16:55启动、16:54:07收尾完成，18273任务步，success=false、failed=true、terminal_reason=assisted-external-evidence-stop。exit0不替代结果。
- 底梁6在2053步CAD近夹：双侧236.857/233.204微米；提起2643、取件清障2732，搬运/悬停、正确放置、全开夹、解除持件、退避和park均通过。7251步建立世界支撑时XY目标误差约.505mm，旧静态目标84/99mm偏差已修。7344步6夹爪最大8.683e-5rad、持件映射全空、native资格12；随后手上升53.10mm，所查115个不同native时刻底梁保持。仅为辅助支撑下的释放稳定，不是纯接触承重。
- 首柱3在11866步CAD近夹：双侧231.961/237.968微米；提起、清障、搬运/悬停及insert00–06通过。原力资格仍分开，不因辅助固定宣称自然抓取。
- 首次失败task18272/physics18275、phase=assemble_00_part_3_insert_07。底梁6与首柱3可见三角面相交参数3.317869微米，超过原1微米数值阈值，触发停止。此数值不是PhysX cooked实体穿深。空间点约[.826153,-.115413,1.014927]m，同clock该pair在柱3的力约[.000193,-3.679965,2.333017]N，模4.35719N；不是靠净力猜碰撞。
- 当时首柱横向误差1.043946mm，仍高于insert07目标8.666221mm/高于最终目标11.906221mm，未越过3mm捕获平面。保持Z/姿态只校正XY的只读几何重算可恢复.814381mm间隙；只校正姿态仍相交。支持横向偏移与冲突有关，唯一动态原因未证。
- 失败窗口11次局部逆解均收敛，左臂最大跟踪误差.006012rad；local→SDK→native所查指令一致至浮点精度，未见旧目标跳变。名义精确路径预检没有覆盖允许/实际约1mm偏移范围；不能再用同名义0碰撞认证新受载轨迹。
- 首柱安装/释放、其余三件原生、最终五件240未执行。失败尾录1200物理tick/5.00000026秒完成，无新policy，保留最后目标；151原生记录含entry+150尾段均有效，三路各新增150帧。尾段柱3仍下移约1.420mm，不能把末帧当插入或释放成功。

## 视频、报告及覆盖限制

- 服务器原始运行`P/runs/A12_beam_full`保留日志、配置、实际轨迹、原PNG和全部报告。全原始归档`P/A12_beam_full_evidence.tar.gz`为117986229bytes，SHA39409905fdef6eb74834f636d0407d02aa4186a485b4b79cdd5129c4f00c043c；该大原始包保留服务器，未声称本机已下载。
- 本机三完整原片在`L/outputs/beam_assisted_20261015/A12_evidence/episode_0000_live_videos/`，各2434帧/81.133333秒，总25433249bytes；服务器完整解码通过，本机/服务器大小和SHA逐一相同，见A12_video_download_verification.json。下载审查包33286044bytes，SHA653e10458e87656957ab27cfcfeb5500ef501ecfc029d54b497171b1c64ae469，包含原片和原始配置/时钟/相机/辅助报告。
- 五个三视角短片在本机同级`A12_key_nodes/`：01_grip、02_lift、03_transport、04_install_release、05_part_3；第五个含首柱失败和尾段。完整解码、帧数、大小/SHA核验见A12_key_nodes_download_verification.json；三视图按原同一物理戳拼接，均保留完整画面。
- 三路各连续2434帧，physics每帧8tick且同帧一致；2435相机trace/4870矩阵读回无错、cleanup完成无更新异常。RTX产品矩阵只在启动实检，后续读回是USD；ReferenceTime与physics是不同起点的钟，不宣称绝对同钟。9140原生边界样本全部有效。
- 已审原帧311/968/1819及904/913/918/1483/1555/2283/2284/2433，夹持样本看起来贴合、壳体完整，开夹/退避明确；微米相交无法由RGB排除。未逐帧认证全片视觉。CAD实际覆盖/最小间隙/errors/skips见B02_mesh_coverage_review.json；仓库未导出实体、采样间CCD、cooked/SDF及尾段CAD流未覆盖。选定native pair不等于完整外力账。
- 核心结论B02_postreview_conclusions.json；独立审查B02_failure_target_review.json、B02_failure_control_review.json、B02_camera_review.json；底梁B02_base_release_independent_review.json。原始记录与报告优先于文字摘要。

## 暂停、恢复顺序及同步

- 当前暂停。恢复必须来自用户新指令；不自动用剩余3轮、不因发现可修问题继续。优先检查真实近接口轨迹的横向/姿态误差范围与下降准入，复用本轮失败姿态验证有足够CAD余量，再做源码绑定离线/原生验收；不要直接提高碰撞阈值或先调夹力。新方法改变先由用户确认。
- 阅读总交接→本文件→固定检查→review_batch/round_ledger/experiment_contract→本轮原证据→长期发现索引。历史A01–A10b在coordination_history/assisted_A01_A10b_handoff.md；本轮清理前全文已保存在coordination_history/before_B02_closure_*.md，不删除否定路线。
- 五协调文件同步本机W/A_L及服务器R/C/旧基线/A；证据JSON、ledger、contract同步P。最终SHA、源码876/887/工具9、运行结束/锁空闲及暂停状态的核验证书为P/B02_final_pause_certificate.json；证书缺失时标sync_pending，不宣称一致。
''')

findings = (L / 'BEAM_FINDINGS_INDEX.md').read_text()
findings = findings.replace('当前B01/A11b准备，完成0/5；旧C876冻结。', '历史B01/A11b启动准备；最终0动作工程失败闭环。最新B02后暂停，完成2/5，旧C876冻结。')
findings = findings.replace('用户要求本轮后暂停，因此该修复及实际随机布局回归尚未实施，', '当时按用户要求暂停；该映射后经B01修复、B02实测正确放置验证，')
findings += '''
### B02闭环：底梁辅助释放新增通过，首柱实际误差超出名义预检范围

- 当前B02/A12已结束并按最新用户要求暂停，B01/B02共2次闭环，B03–B05延期；v16b887源/9关键工具原生前后零差。完整五件和全程视觉未通过，formal/assisted success=false。
- SDK发布既有物理结果到USD解决617mm初始化画面错位，44body达到约3e-8m位置差，q/body/clock/计数/history未变；真实三图门通过，不增simulate、不写body状态。比单看相机矩阵更完整。
- 底梁真实目标映射修复已验证：辅助近夹、提起/搬运/正确放置、全开/解除持件、退避/park新增通过；7344全6关节近0、native12、115后续不同native时刻稳定。世界支撑辅助与纯物理承重分别陈述。
- 首柱辅助近夹/提起/搬运及insert00–06通过；insert07在task18272/physics18275因底梁6/柱3可见三角面相交参数3.317869um超过原1um数值阈值停止。实际XY误差1.043946mm，尚高于该路点8.666221mm。冻结pose只校正XY即恢复.814381mm间隙；仅姿态校正仍相交。横向误差是相关因素，唯一动力学原因未证明。
- 同clock该pair在柱3载荷约4.35719N，空间点和body姿态已对齐；11次IK收敛，所查local/SDK/native指令无旧目标跳变。名义7423状态/54mesh零相交不能认证实际/允许误差范围。后续恢复优先实际误差包络和近接口下降准入，不能只增epsilon/调力。
- 三原片各2434帧及五节点短片已下载SHA同、完整解码通过；首柱失败尾段5秒观察不等于安装成功，尾段仍保留目标且柱3下移约1.420mm。采样间/仓库/cooked/尾CAD缺测保留。证据B02_postreview_conclusions.json、三独立审查、B02_mesh_coverage_review.json及视频核验；当前协调状态以B02_final_pause_certificate.json为准。
'''
(L / 'BEAM_FINDINGS_INDEX.md').write_text(findings)
agents = (L / 'AGENTS.md').read_text().replace('最新用户2026-10-15要求本轮B02完整收尾后暂停；不启动B03，覆盖此前五轮连续授权的后续部分。', '最新用户2026-10-15要求本轮结束后暂停：B02已完整收尾并暂停，不启动B03、不继续修复或仿真；覆盖此前五轮连续授权的后续部分。恢复须用户新指令。')
(L / 'AGENTS.md').write_text(agents)
checklist = (L / 'BEAM_REVIEW_CHECKLIST.md').read_text().replace('2026-10-15最新用户要求当前B02闭环收尾后暂停，B03–B05延期；下列固定检查继续有效。', '2026-10-15最新用户要求本轮结束后暂停，B02闭环已结束，B03–B05延期，不继续修复或仿真；下列固定检查在用户恢复后继续有效。')
(L / 'BEAM_REVIEW_CHECKLIST.md').write_text(checklist)
print(json.dumps({'closed': 'B02', 'completed_cycles': 2, 'paused': True, 'next_native_run_authorized': False}))
