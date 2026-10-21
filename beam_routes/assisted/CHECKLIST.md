# 辅助路线附加检查

先读根 BEAM_ASSISTED_HANDOFF.md 与共用 BEAM_REVIEW_CHECKLIST.md；本页不定义授权。

- 允许的持件fixed joint和安装世界支撑按本路线交接执行；不写零件位姿/速度、不清速度、不切kinematic、不关闭碰撞。辅助稳定不作为自然抓稳/承力证据。
- 可见双侧pad表面间隙5—250um、无交叠及12独立native tick后按实测位姿捕获。捕获6DOF关系固定；分别检查腕FK校准、捕获后累计/瞬态漂移、最终持件预测残差，不每tick重锚掩盖漂移。
- 装配右臂全开→近夹底梁→视觉扶持→最后撤回；同run/epoch最近最终发布目标不能丢失或过期。全部后续件、完整双臂/夹爪/持件、实际三视图一起验；视觉扶持不冒充承力。
- 严格插入准入150um/1mrad/12独立tick；模型残差100um/1.5mrad。固定世界冷轴参考、120um有界横向积分与500um实际冷高度门按v22原值，不因预检调整成功门。合格姿态应能前进，危险状态应被拒，不能全hold伪通过。
- 全阶段节点、相邻控制边及guard首次启用均列覆盖。使用真实context跨tick、真实非零published−actual偏置、实际关节演化及SDK前二次限幅；derived float噪声不清缓存，真实身份变化按合同清缓存。
- 当前改动、证据与待验范围只读[当前交接“本轮变更与验收边界”](../../BEAM_ASSISTED_HANDOFF.md#本轮变更与验收边界)；当前状态/次数/执行权限只读唯一账本，本页不重复维护。
- 左右腕相机各绕自身光轴180度；启动后实际查看本进程同钟三原图，确认夹爪在画面下方再ACK。不能用旧图、后期翻图或镜像替代相机改动的实际成像核验。
- 原右臂撤离后，左home→右肩`-0.24rad`定向抬离→右home。抬离时其余五轴保持入场实测值；初始目标读取实际配置。每段实测home/抬离到位需8个独立native tick，最终双臂home误差≤0.01rad须与原五件合同共同保持240tick，不能按请求或发布目标判到位。
- 空手回位也须在最终限幅后以同钟双臂实际状态逐批检查完整54/self/完整夹爪、全部已装件及导出环境，不省略静止臂，不新增抓取豁免、不放宽原交叠判定。名义路径与邻近候选扰动证据分开，不能称完整误差包络或连续CCD证明；D28真实published−actual及最终SDK/native链已在存档范围核验，主execution后审true，299存档批/1495有限full54/self样本通过；只按已记录范围成立，终态raw缺口及原final_home_review=false保留，见[D28结论](../../outputs/beam_assisted_20261015/D28_conclusions.json)。
- 正常终止后只读native采集边必须覆盖最终同钟五件/双臂证书，避免D26完整独立raw native早7tick的缺口；D28主五件/home240通过，但独立raw末帧早6tick、action/三视图/entity早7tick，原final_home_review=false继续保留，见[D28结论](../../outputs/beam_assisted_20261015/D28_conclusions.json)。60/240个raw时钟索引不补为240个实际状态，299存档批/1495有限样本不冒充逐步覆盖。D26第四件insert02/04/06/08及第五件insert08的准入采样缺口同样保留；本轮stride8若缺准入tick仍须明确列缺测，不以阶段推进补造连续读数。
- 共用夹持调用覆盖以phase+robot+object及控制边枚举，不能按object去重；包括底梁close、底梁再次视觉扶持和未达part0。真实居中后缓存、最终SDK归一化和稳定/瞬态误差分别建模，旧版真实已通过入口作正控；启动后补审不得回填启动前覆盖。原双侧5—250um/12tick门不变，新抓取关系须由实际全实体guard重新审，通用自动适用域检查仍未实现。
- 发布到SDK严格比较同语义目标；SDK到native target先核精确float32转换，不能用任意放宽公差掩盖差异。失败周期空/重复batch与新动作分开，尾录还须核两臂native目标逐样本不变。
- 收尾后再读取最终collect_results与原报告，编码期间临时空列表不判为报告缺失；status/exception/terminal_reason和所有成功门一起核，exit0或task.failed=false不代表通过。检查普通失败尾录是否实际执行，钩子异常绕过尾录则明确记缺测，不合成结果或补造时钟。
- ACK/STOP使用已有共享原子publisher，绑定当前run/图SHA/physics且拒重复；先就绪后receipt，确认第一动作及CAD/entity计数增长。保留原报告，以同epoch/index/time审phase改标缺口。
- 已有工具入口 `outputs/beam_assisted_20261015/tools/`：`run_offline_checks.sh`、`build_delta_gate.py`、`index_run.py`、`review_run.py`。先按实际任务查工具参数，不每轮复制检查器；旧六门需补生产实际轨迹适用域后才可恢复。

- 独立断点恢复实验入口、松散实现及专项测试已按用户指令移除；正常从头辅助执行不得依赖CP实验目录。训练模型checkpoint/续跑、普通路径检查点及启动/native观测快照继续保留；清理身份与回归证据只读当前交接。
