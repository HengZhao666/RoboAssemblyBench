# Beam 当前交接

更新：2026-10-16。D03/A19已结束并收尾暂停；D批已启动3/10、闭环3。最新用户要求本轮结束后暂停、分析原因/流程/时间；D04—D10未启动且不授权。next_native_run_authorized=false。旧C01—C04及C05未执行历史不改写，旧物理v201/source876冻结。不自动公开或更新PR。

## 权威与封存

- L=/Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench；P_local=L/outputs/beam_assisted_20261015；A_L=P_local/assisted_repository仅同步副本。W=L/submission/beam_upstream_integration_2026-10-10，branch codex/beam-runtime-repair-20261011，HEAD431050cda779b9a31cae21c548c761705e3b9482，既存脏树保留，不reset/clean/add-A。
- SSH=pcl-server；R=/home/user/beam_integration_20261010_upstream6f57dae；P=R/beam_assisted_20261015；权威辅助源A=P/assisted_repository。C=R/candidate_repository为旧物理876；/home/user/RoboAssemblyBench仅协调文档副本。路线physical/assisted保留，当前只修assisted。
- 当前候选v22源889清单SHA653ab091d35b99b97f50da9c16f7766622acdcaa9ff815341318357ebaeacf6a；关键工具20清单SHA5ad3f3f08ac507124089e01a767019a3e706312c47da1c3a3345fa7ce71eb2a3。A19源后验SHAe658c3ba2451ca01bd0106e2f047369da5e9491b862a8e5d8345dab74164960e，889/20前后零差；最终逐SHA另核冻结物理876均一致，原生进程0，见D03_final_runtime_state.json。
- P/versions/assisted_v22_before_A19_beam_full.tar.gz，5881257bytes，SHAedc6e6ce3bfbc117254628129b7bc849fb0b751778376523bd74e0e28ccc9c3b；工具archive SHAa734528a5145cda6a54f79a11f14d435111f8f8896607027e71ce6a9eb7e4143。v21/v20/v19及全部旧源/工具/日志/视频保留，准确路径见对应sealed_version；当前v22保留为未完成候选，不冒称验收版本。

## D03结果与错误原因

- 首柱insert00于16274完成，十段插入及release_and_lock于17326通过，17739转入第二柱。首个已存12tick准入14264，横向112.702um；原150um/1mrad/12tick未放宽。冷世界参考最大已存位移353.575um<500um，无实际冷高度越界事件。首柱已完成安装/松夹/退避，不等于全五件成功。
- 第二柱assembly_clearance在23272被完整可见CAD监视拦停；结果23273steps，terminal_reason=assisted-external-evidence-stop。fabrica_beam_2与右臂wrist_2_link实际三角网格相交.414830579mm，违反无穿模要求；本轮failed=true、assisted_success=false。不是超时，也不是首柱冷准入失败。
- 6056已存最终发布/SDK逐值相同；第二柱清障停止前模型残差峰仅2.191um/.302492mrad。碰撞后failed帧增至204.658um/3.055mrad，不能倒因果误诊成原先抓取失稳。失败快照73接触对未独立覆盖该腕/零件对，不能据此声称无物理接触；空间CAD相交已足够拒绝。
- 旧名义路径同阶段264点最小间隙8.576mm，实际失败姿态与按位置最接近的名义点仍差19.114mm/.762579rad（43.69度）。nominal_bimanual.move用同一f线性平移+Slerp；生产控制独立积累平移/旋转并经过实际关节限幅/跟踪。配置目标+6cm已在实际源中，问题不是漏部署抬高参数。AST/文件SHA一致只证明计算/存档不变，不能证明实际轨迹落在名义走廊内；本次安全复用范围判错，已有“理想插值不能替代生产路径”规则执行不到位。
- 同实际起点下，最终已接受SDK目标的离线FK预测相交2.724mm；这是预测，不是宣称实际穿透达到此值。下一修复应保持右扶，分开第二柱原位抬高与横移/旋转，给右腕留完整持件扫掠余量；暂停后不实施、不启动D04。

## 后审、视频与范围

- 2909 full54/self实际采样，745项数值表面接触保留，发生上述非目标失败。右扶2093样本独立同钟/三相机/三原生实体逐值绑定通过，视觉扶持不冒充承力。原C_support_summary退出4，23272的phase改标failed导致按phase缺1native；原报告保留，support_clock_join_review按同epoch/index/time证明2093/2093同姿态，不改写原报告。
- 失败5秒/1200physics尾录、150native、三相机各150尾帧完整。原始日志/PNG/归档P/runs/A19_beam_full与P/A19_beam_full_evidence.tar.gz保留。原始后审/原扶持报告/独立诊断均在A19_evidence。
- 本机三完整原片P_local/A19_evidence/episode_0000_live_videos，每路3059帧/约101.967s，总30153320bytes，服务器全解码0/本机逐大小与SHA一致。7节点P_local/A19_key_nodes：底梁抓取/提起/搬运/安装释放、右扶、首柱完整装配、第二柱失败窗口。D03_transfer_verification.json核定。
- 同钟原图8896、18944、23272及三视角各8视频阶段样本已直接看；夹持/右扶外观贴合，但右腕干涉可被遮挡，不能以画面不明显豁免CAD穿模。未逐帧人工认证、未导出warehouse/采样间CCD/PhysX cooked差异仍缺测。第二柱后续、件1/0及共同240最终门未到达。

## W04.4与耗时

- 流程执行详见BEAM_WORKFLOW_LOG.md及BEAM_REVIEW_CHECKLIST.md。保留一轮瓶颈审查、差异检查、冻结后整源检查一次、六启动门、一次正式原生、一次不可变索引/后审/媒体同步；不靠重复五遍广审代替反例。每两次正式尝试复盘，新瓶颈/阶段切换提前触发；本次D03新搬运瓶颈已触发复盘。
- 几何安全复用必须绑定实际入口q/抓取6DOF/已装件/右扶及生产最终命令整段包络；仅源AST/config/pathSHA不够。旧理想插值仅名义参考。下次恢复前先用23272反例重放真实生产搬运全阶段的平移/旋转不同步、实际q演化、缓存交接和SDK前二次限幅；不能三锚点/25固定actual回放替代全程。
- 新工具先语法及小输入契约，ready后再下载/receipt；异常也记结束与耗时。SSH断开先核远端完成状态。本次suite/config已完成，未因255重跑。计时合并两主机原流、去重，不覆盖另一主机新增事件；跨PID手工阶段按开始时刻匹配。
- 本轮已测：修改1097.770s；预检工具/检查/启动绑定999.937s；整源421项201.200s/2既有warning、四配置35.147s、7演化偏差模型+7基线/448 full54/self65.767s。正式原生2500.615s（41m40.615，含初始化/91.937s ACK等待/尾录/编码）；顺序后处理61.212s；媒体下载57.787s、核验.338s；三路生产编码唯一合计62.364s。初版分析把两个元数据别名重复计时，原版保留，reviewed版为准。
- 这些是inclusive或并行区间，不能相加。总墙钟从D03修改06:29:39Z起至本次结论，见D03_conclusions.json/timing；不含更早D02诊断，最终同步另记。新检查器准备、临时工具返工和后审归因仍占用时间，不能说每次调用都有装配进展。缺单独起止的调用不补造；首个过早receipt异常缺end，不是仍运行。
- 六文件本机W/A_L12副本及服务器R/C/旧根/A24副本按本次闭环逐SHA核验，最终以D03_final_pause_certificate.json与batch_D_20261016/D03_final_certificate_sync.json实际结果为准。未核验项标pending，不以文档文字代替证书。

压缩前完整六文档：coordination_history/D03_before_pause_compaction/；更早W04/D02/C批历史、机制和否定路线均保留。D01修复pickup，D02修复自由搬运交接，D03完成首柱但暴露第二柱真实搬运走廊失配；不能将局部进展写成完整装配成功。

W04.4实施边界：本轮更新了流程/固定检查和适用域要求；v22控制代码及旧六门尚未补齐自动的实际搬运轨迹域认证，不能把文档更新当作已实现。恢复前必须先以23272反例实现/验证这项检查。当前保持暂停，不开展下一轮。
