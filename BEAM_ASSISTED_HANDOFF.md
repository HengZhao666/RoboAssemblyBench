# Beam 辅助路线当前交接

2026-10-21，W06.0。本文为当前短入口；实时状态、次数及权限只读[唯一账本](outputs/beam_assisted_20261015/repair_batch_D_20261016.json)，本页不提供启动授权。本次仅流程/经验整理与实录性能分析，不改控制代码、不启动仿真。修复其他Fabrica辅助任务直接读[新任务入口](FABRICA_ASSISTED_REPAIR_HANDOFF.md)，不加载本页历史。

## 本轮变更与验收边界

最近一次封存结论为 [D28/A44](outputs/beam_assisted_20261015/D28_conclusions.json)：v35普通从头完成五件辅助装配、三段回位及原五件/双臂home共同240tick，原success/assisted_success/motion_success=true；主execution后审true/errors=[]。**严格独立终态仍缺raw，final_home_review=false、accepted_full_assembly=false、all_postreview=false**，不能称全部后审通过。结论不覆盖旧原件，当前权限以账本为准。

- v35仅修真实框架env0/env1逻辑名/运行时名映射两源文件，66项受影响检查通过。原A43实际构建失败原因及配置保留于[D27结论](outputs/beam_assisted_20261015/D27_conclusions.json)；v35资格只引用 `D28_work/alias_fix_qualification.json`。应先走真实worker构建→框架映射→生产预检的无物理冒烟，不以单函数测试冒充。
- v34原新增两腕光轴180度滚转，原右臂撤离后追加“左home→右肩抬离→右home”。A44实际三图确认两腕夹爪在画面下方；末段从实际配置读home，右肩 -0.24rad且其余五轴保持入场实测值。动态结果只引用A44；原名义筛查/响应代理资格在 `D27_work/source_qualification.json`、`return_geometry_delta024.json`、`formal_gate_spec.json`，有限离线样本不证明动力学、连续CCD或完整扰动包络。
- A44实际59567步，task59566/native59570结束；左右六轴最大home误差0.0051313592121/0.00480947596952rad（门0.01rad），三段589/112/1450tick，共同窗口240/240。完整54/self/已装件的299个存档命令批、1495个有限样本通过，不冒充逐步独立覆盖。运行后source889/tools20差异均为空。
- 证书native59570缺同钟独立raw：最新after-physics/observation59564早6tick，末action/三视图/entity59563早7tick；240窗口仅60个raw索引，其余180不回填。独立原件：[final_home_independent_read_review](outputs/beam_assisted_20261015/D28_work/final_home_independent_read_review.json)。相机7445组对齐、native34044有效记录和主execution通过均不能替代该缺口。
- 原runtime capsule442项、stage capsule376项是粗近似提示；原sequence含10条release_without_attach警告/errors0，其模型未识别assisted生命周期。原报告和有限精确CAD复核保留，不能写成“原报告无警告”或将胶囊提示直接当mesh碰撞。

若后续获授权，优先修终止/采样顺序和所需窗口独立raw采集，与其他可分别核验的无冲突缺陷组成一包；成功门不放宽、旧false不回填。通用流程见[共用技能](skills/simulation-repair-loop/SKILL.md)，具体减步/提速候选见[性能分析](beam_routes/assisted/PERFORMANCE.md)。这些均尚未实施/动态验证。

## 权威、候选和回退

- 本机L=`/Users/zhaoheng/Desktop/PCL work/Beam修复/RoboAssemblyBench`；P_local=`L/outputs/beam_assisted_20261015`；A_L=`P_local/assisted_repository`为副本。W=`L/submission/beam_upstream_integration_2026-10-10`保留既有脏树，不reset/clean/add-A，不自动更新PR。
- 服务器`pcl-server`；R=`/home/user/beam_integration_20261010_upstream6f57dae`；P=`R/beam_assisted_20261015`；权威A=`P/assisted_repository`，无Git。原始运行在`P/runs/<run>`；v33已验私有候选保留`P/D24_work/candidate`，本轮资格候选在`P/D27_work/candidate`，v34及其v35映射修复均按下列封存保留。
- 当前v35为889源/20关键工具：源清单SHA`6fb47c35bccf5be5fdd854d17c1bfe825dc59d9722708c15510e1217d9fd4fd7`；关键工具清单SHA`1b17561c813c4c77b0f5d383d62058c801df7ed64a704beab1e9af8d03af5218`（工具未变）。封存`P/A44_beam_full_sealed_version.json`，源包SHA`677a9855adf3bd9200940032c628c9917d019a00f0cc3c7e8a350a5f7d494bd1`、工具包SHA`b196332c33a82647608245c60271820b99e844f941683a5f330ad3427830d574`。资格`P/D28_work/alias_fix_qualification.json` SHA`751d3b1a4c8c74772219944d8d0192d143d9d7f1eb902452481a2a0a7b5c9249`；相机实图、三段回位与原五件/home共同240tick已由A44主判定达到；独立末帧raw缺口及原false保留，见D28结论。
- 历史v34/A43启动失败版为889源/20关键工具：源清单SHA`75c627bd7ea35f392d39ec58e8ef5ced2cc627d6e06c4465465a11340e9c88d6`，关键工具清单SHA`1b17561c813c4c77b0f5d383d62058c801df7ed64a704beab1e9af8d03af5218`。相对v33改10源（含四测试与一个既有joint技能注册）和worker/workflow/geometry三个关键工具；原夹持、约束、solver、持件servo与成功阈值不改。源包SHA`1066f94b0783c8049cee1ed8a1a80984d3e3f8ec87df33f9ae354e2faa415adc`、20工具包SHA`1fb75e957c1cafefa4469eec290667572219bbf6f01672424de1e6b7792d79d4`，封存`P/A43_beam_full_sealed_version.json`。v33十源/清单回退在`P/D27_work/before_authority`，工具原件在`before_tools`；本机旧副本保留`D27_work/before_local_authority.tar.gz`。四布局505实际签名`b38c34c0b0bb61b8b9b66315778566fc74595fffff0839f1b2d164e6baf7eeb8`。资格未宣称native或完整装配通过。
- 上一版已冻结部署v33，889源清单`P/tools/assisted_manifest.json` SHA`476bee3ba19a5c3daa107e6058080e6be9e5715dc6dede48ee34c401477e22e7`；20关键工具清单SHA`4ca6f0c32f07f541fd7b56b5f6661266eb6df4a090f284f76b03ba3ae34dbc17`。相对v32只改compiler/servo/原测试三源：第五件复用共用分段搬运，turn只回撤实测多余向上超前；169受影响检查、四布局精确投影、A40完整搬运与A41第五件插入释放短筛已验。原solver/速度/模型/抓取/成功门不变。源包SHA`05d69f28e24c37c2cfefccc66655d57231cef6a3fc4ed68b00cd9ae27034b5ed`，工具包SHA`3f3788d36058f45dbb4256827e810307d3d5ecddf98796b8124ff043c588412c`，封存见`P/A42_beam_full_sealed_version.json`；v32三源/清单回退在`P/D24_work/before_authority`，本机在`D25_work/before_local_authority`。身份按清单核对，不按目录/Git猜测。

## 不变合同与执行入口

- 原双侧可见pad gap5—250um、无交叠、12独立native tick后按实测位姿创建dynamic fixed joint，6DOF捕获关系冻结。生产不写零件位姿/速度、不清速度、不切kinematic、不关碰撞。辅助固定不称自然抓稳或承力。
- 右臂全开→近夹底梁→视觉保持→最后撤回，保留同run/epoch最新最终发布目标。严格插入150um/1mrad/12tick；模型100um/1.5mrad；冷世界轴、120um横向积分、500um实际冷高度、原速度和24tick换段均不放宽。
- 安装上方3mm世界支撑，六夹爪实际全开≤.05rad后按原≤.1mm/.2mrad每物理步缓落。最终五件夹爪释放（安装世界支撑仍保留）、原目标3mm/.04rad、共同240独立native tick及原全序列完成才接受。
- 每轮读`BEAM_REVIEW_CHECKLIST.md`和`beam_routes/assisted/CHECKLIST.md`。固定工具在`P/tools`：真实回放`replay_assisted_transition.py`，独立短筛`assisted_probe.py`/`launch_assisted_probe.py`，短筛后审`close_assisted_probe.py`，普通从头入口`launch_reviewed_run.py`，完整后审`review_run.py`。ACK/STOP始终用共享原子publisher；实际看过本进程三原图后才ACK。
- 六项门绑定当前源/工具/配置和历史适用域；启动后必须确认首动作与native/entity/CAD实际增长。完整最终限幅后双臂、完整夹爪/持件扫掠不省略。原首失败及5秒物理尾录、缺测、原报告覆盖和三视图同钟不省略。
- 每两次原生复盘，新瓶颈提前触发；两次无改善停止同类局部试错，取得判别证据后再走下一机制。可分别验证且无干扰的修复可以合并，不重复全仓回归。不得把短筛或exit0写为全装配成功。

## 媒体、性能与历史

A44本机三完整原片在 `outputs/beam_assisted_20261015/D28_work/A44_media/episode_0000_live_videos/`，各7445帧/30fps/248.166667s；服务器完整解码、本机60传输文件SHA、三原片9点AVFoundation播放及7组同钟三图检查通过。回位短片289帧/9.633333s。媒体只引用 [D28_local_media_review](outputs/beam_assisted_20261015/D28_work/D28_local_media_review.json)；原片帧数不补终态raw缺口，旧A42视频不替代新视频。

运行步数/分段及耗时只读[PERFORMANCE](beam_routes/assisted/PERFORMANCE.md)和对应机器证据；Kit.update合并物理/渲染，尚无独立物理GPU时长或提速对照，父子/并行计时不能相加。共用文档版本和同步证书见 `beam_routes/registry.json`、`beam_routes/docs_manifest.json`，旧实验结案证书不覆盖。

本次整理前27份完整文档按旧同步证书逐SHA保存在 `outputs/beam_workflow_transfer_20261021/before_documents.tar.gz`，清单在同目录 `before_manifest.json`；其中原 `BEAM_ASSISTED_HANDOFF.md` 保留全部历史路径/回退/媒体。只在需要旧原因或回退时检索[辅助FINDINGS](beam_routes/assisted/FINDINGS.md)、相应 `D*_conclusions.json` / `*_sealed_version.json` 或提取封存原页，不默认读全历史。D26/A42为先前五件主成功基线，同样缺完整独立raw末帧；旧短场景先前零件为已安装初态，不能计为本轮从头装配。

已停止的断点专项不恢复：松散实现/入口/专项测试已移除，等价恢复没有得到验证。清理边界见 `outputs/beam_output_cleanup_20261019/cleanup_summary.json` 和 `outputs/beam_checkpoint_code_cleanup_20261019/completion_summary.json`；普通训练checkpoint/路径节点保留。通用断点方法不等于已有可用恢复能力或运行授权。
