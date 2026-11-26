# Stool 原合同与辅助修复证据

原合同及修复包以用户选择的服务器辅助基线为准；当前权限和暂停只读 [execution_ledger.json](execution_ledger.json)。源码、配置构建与动态装配分别给结论，不把接口测试或初始化检查称为装配成功。

## 最新增量合同：持续扶持、完整安装与原报告几何复核

授权、计数、暂停及下一动作只读 [execution_ledger.json](execution_ledger.json)。本节说明实际实现和证据边界，历史结果仍由各轮原件维护。

- **调度与原门**：`stool_continuous_base_support_v3`为单次连续生命周期。原底件2安装、实测rebase及72 native tick释放/开爪后，任何左臂取件前取得右扶持12 native tick资格，覆盖八件全部166个原左阶段。末件释放后右臂实际开爪12 tick，再验九件原released/static/P/R/速度及独立72 tick终态。最终183阶段/max799792；原顺序2→3→7→6→4→5→1→0→8、九件目标和criteria、九次抓取/安装及一次rebase保留，没有新增最终home/park门。
- **右扶持位置与联合路径**：右臂采用前视画面左侧姿态，底件释放后经四路点入场。抓取入口、左臂完整安装/释放/原退避/park与右姿态共同检查；S22及S23已实际跨过S21双臂候选干涉点。原7mm/.1rad区域、250µm可见指垫近接上限、125µm闭合预测目标、500µm对中领先和12连续native tick不放宽。
- **命令与原生身份**：资格后保持实际SDK接受的完整右臂6q和开度，每native batch联接CAD、SDK及独立raw；不重捕获或覆写底件位姿。右底件完整闭合预测沿实际action→bounded action→全12q出口，包含手臂及夹爪联合终点；外层完整扫掠、限位、步长/跟踪限幅及SDK仍必经。预测不冒称已执行。
- **S23实际控制修复**：左臂任务专用PD改为160000/5656.85424949238，右臂同值；effort限制保持。此改动消除S22 part5约0.597mm的稳态对中偏差，使原闭爪门实际推进，并非放宽资格。part0改用已核实的侧面抓取位置和相应空手接近路径；九件最终目标、250µm/7mm/.1rad/12 tick及安装终态门不改。S24复用同一物理计划。
- **连续SE3和完整检查**：保留原有界关节及持件object-SE3路由、同R逆完整XYZ、统一位置/旋转进度、.25mm/.012rad增量及原领先界限。完整双方臂、全部夹爪、持件、已装件和环境参与最终发布检查；离线模型与原生执行分别给结论，不将首个已通过路点扩张为整链证明。
- **观察服务与首动作**：真实RigidPrim及contact服务预热必须早于启动图像/raw基线，运行期只读封存服务。缺失、失效、epoch/view变化及重初始化请求显式拒绝；不因metrics捕获异常继续送SDK。robot缓存/fallback在同一exact-path出口登记原生reader。完整初始化后仅计算一次原首策略动作及完整guard，SDK仍0、全部原生态不变；冻结同钟三图并ACK后返回同一动作，不重复dry或重算推进阶段。
- **S24原报告修复**：Stool运行报告保留全部原胶囊事件，原分类、距离、总数和阈值不改。仅对四类已知端点胶囊—板面事件记录完整刚体矩阵及native钟；阶段见证还含原校准关节图、current/target全12q及fraction。独立后审联接同钟raw并重算完整关节图，再用完整可见及已声明碰撞端点网格的正分离平面逐条解释代理事件。原事件及旧S23 false不回写；未知对、证据截断/缺失/过期、图/矩阵不符或不能证明正分离仍失败。该证明不扩大为连续CCD、cooked/SDF或接触力认证。
- **辅助与力的边界**：保留实测安装世界支撑；右臂SUPPORT不新增右臂—底件连接，`force_support_claimed=false`。固定命令、几何近接、零件稳定和真实承重分别报告，0N/缺力证据不能证明摩擦或承载。取得12 tick、逐tick持守、末开爪12 tick及九件终态72 tick各自需要原raw/SDK/CAD同钟联接。

S22为5/9；S23实际9/9、完整183阶段通过，但严格后审因原代理事件未消解及摘要截断仍false。S24重复完成原九件及完整183阶段，14项独立后审全部通过；原被动事件全部保留，并以同钟原生状态和完整关节图逐条复核网格正分离。持续扶持53184 tick覆盖全部166个左臂阶段，末右开爪12 tick及九件终态72 tick均通过。物理控制方案沿用S23，辅助世界支撑仍保留，不宣称右臂独立承重。 原件与局限见 [S22/S23复盘](../../outputs/stool_assisted_20261021/grant03/S22_S23_pair_review_20261027.json)及 [S24复盘](../../outputs/stool_assisted_20261021/grant04/S24_round_review.json)。历史失败及缺测不回填。

<!-- latest-increment-end -->


## G03 首轮前审查与修复包

原版 GitHub Beam 的最初引入、十月前最后主线与辅助修改版的差异见[原版对照](../../outputs/stool_assisted_20261021/grant03/review/beam_origin_comparison.json)。S03/S04 毫米级指垫间隙来自仍在使用的宽松接触盒资格；本包按 stool 真实指垫内面、朝内法线、原抓取区域和连续12 native tick 接入闭爪/居中与 attach/持件复核。CAD近接上限取本任务原最小位置步长250µm，目标为其一半125µm；不能称受力或摩擦抓稳。手腕与TCP分别记录，原抓取元数据按已核原生手腕frame解释；其余七件首次物理可达性仍须逐件观察。

- [CAD审查与复用](../../outputs/stool_assisted_20261021/grant03/review/cad_grasp_review_receipt.json)保留九件真实网格合成反馈、过闭反例及旧2mm/8mm间隙拒绝。最终控制差异以旧运行真实跟踪偏差做连续合成反馈验证；不是原生动力学重放。SDK最小开度从实际配置模型解析，见[默认值修正](../../outputs/stool_assisted_20261021/grant03/review/cad_default_resolution_fix_receipt.json)。
- 原阶段报告接完整夹爪关节预测；逐prim区分无碰撞声明、显式disabled与不支持形状；完整事件流须经SHA/计数/摘要后缀守恒核验，才能解释摘要截断。旧报告不回写，未知/不支持/缺失仍拒绝。
- 失败钟强制独立raw不受stride限制；现有运行/收尾记录新增阶段与策略/媒体/后审计时，重叠分别说明。完整碰撞查询缓存只复用单次调用内两完整变换字节一致的FCL结果，全部对象对、采样与接触重分类保留：[等价与计时](../../outputs/stool_assisted_20261021/grant03/review/integrated_cache_result.json)。离线查询域提速不能写成整轮提速。

每轮仍需从头运行、一次固定后审及三路完整原片；按[清理清单](../../outputs/stool_assisted_20261021/grant03/review/cleanup_manifest.json)逐步删除无价值派生输出，保留有用原件、回退与身份封存。新轮实际结果只按账本及相应原运行记录填写。

## S13 transport与资格边界

余下6个transport_hover使用完整object SE3、同R逆全XYZ与fresh最终SDK参考。既有7个task-derived125µm cap/1⁄12 XY bias lift、14个clearance SE3保留；原179阶段/原目标/capture/九件criteria/max_steps786216不改。原250µm visible-pad近贴合、原抓取位置区域与最终72native稳定门仍独立生效。

明确Stool adapter下，stage原candidate按冻结原contact policy分类，保留每条raw候选与原几何/采样；非有限/未知分类fail closed，实际非豁免负距离仍collision。旧S12报告不回写。part6实际新transport及插入/释放完成，原criteria4/9；part4 close的local visible-pinch被原抓取位置区域门拒绝，原始数值与时钟见[S13复盘](../../outputs/stool_assisted_20261021/grant03/S13_round_review.json)。本轮完整装配仍失败，权限和暂停只见账本。

## S11 统一goal进度与实际2/9边界

仅helper把原position/R完整goal推进改为统一归一化fraction；S10步长、lead、原目标/完成门、固定capture、全guard及工具不改。完整constructed路径按原oracle完成、原代理域无候选，原report-only异常与JSON恢复分别保留，不构成native成功。

实际零件3插入/释放完成，base2与part3共2/9原criteria passed；随后part7 lift发生实测native位姿下fixture–part7可见CAD重叠。原报告覆盖门本轮通过，旧S10 proxy false不回写；新native-pose CAD失败仍停止且未发新SDK。此新零件瓶颈与完整九件成功门分别处理。[S11复盘](../../outputs/stool_assisted_20261021/grant03/S11_round_review.json)

## S10 完整object SE3 clearance的实际边界

仅原part3 pickup/assembly clearance接完整object SE3：位置/旋转共用回退比例，冻结同一抓取R反解TCP完整XYZ；以fresh final-SDK6q的FK为入口，entry/越包络/正常无解保持原完整目标。使用原插入0.25mm/.012rad增量、3mm object/TCP与.036rad旋转lead；显式超时由4800扩到48000、总预算181416，原目标与完成门不改。数值精度门与物理安全门分开，最终guard后的float32/clamp由下一fresh receipt承接；异常仍failclosed。

实际回归lift/pickup通过，assembly_clearance未完成，安装件数未增。首失败是future f1双臂可见/碰撞网格重叠，同钟native scene clear；位置/旋转目标归一进度失衡，不能将转动更多或较小抓取残差称为任务进展。原proxy/未分类报告无explicit resolution，原auditfalse保留；2秒尾段与终态/首失败钟分开。[S10复盘](../../outputs/stool_assisted_20261021/grant03/S10_round_review.json)

## S09 有界XY反馈与新失败边界

实际S09只对part3 lift接入125µm cap、1/12每native tick的XY reference bias及12.5µm数值命令精度条件；其他八件保留S08v2，3mmlead/R/Z/completion/timeout1200/max95016未变。命令精度不是物理成功或碰撞阈值；有限FCL未调用actualLula，正式反馈未独立记录内部tolerance入参。未运行的lead-only原提案和旧配置差异保留在重规划收据，不能冒作本轮控制。

原runtime已通过part3 lift与pickup_clearance，随后在assembly_clearance失去可见指垫CAD near-fit资格；generic异常没有exact gap/ROI/normal分量，安装件仍只有base2。尾段独立raw已按native index stride8获得60条完整41体/9件/双臂12DOF实际样本，旧S08缺测不回填。后验独立原函数复算识别左gap超原250µm门0.882909µm；此静态诊断和frozenH-P反事实不构成动态修复，不放宽门。原审计false与九件最终门不改。[S09复盘](../../outputs/stool_assisted_20261021/grant03/S09_round_review.json)

## S08 object-XY v2 的实际边界

S08改为绝对object XY目标，并用冻结抓取变换反解期望手腕，R/Z与completion保持原样。5个有限模型和3次完整guard正反例通过，但native0、actualLula false。正式动态已恢复底件2完整阶段，仍在零件3抬升的实际原生位姿CAD检查失败，未超过S05；不能称整体抬升修复或九件装配成功。失败同钟raw完整，终态5450与失败5449分开；attach近接资格不等于力/摩擦证明。独立2秒物理尾段已在S08运行，不改此前5秒原件。详见[S08复盘](../../outputs/stool_assisted_20261021/grant03/S08_round_review.json)。

## S07 继续验证与未解决瓶颈

S07仅修改抬升姿态来源，真实动态仍在底件2抬升的候选扫掠终点被拒绝；未提交新SDK，不将预测5.645921µm重叠称为native穿透。静态R反事实不能替代动态修复通过。后续接实测TCP、实际IK目标、最终SDK关节与FK完整XYZ/R以及持件旋转pivot做同钟复盘；九件最终72tick门仍未访问。S07原5秒物理尾录与后续2秒工具候选分开，原报告和auditfalse不改。[S07复盘](../../outputs/stool_assisted_20261021/grant03/S07_round_review.json)

## S06 native 横向反馈的实际边界

S06 仅对九个持件world-Z抬升阶段接入native横向反馈：从已提交的最终完整XYZ/旋转目标继续增量，保留原目标、原步长/包络及同tick去重；其离线通过不等于动态可达。实际在底件2抬升候选终点即被预测CAD重叠拒绝（未SDK），比S05更早失败；完整失败同钟raw已保存，须据此重新规划，不能沿用旧离线成功结论继续同一试跑。最终九件72-tick门仍未访问。终态图native1306与失败1305区分，原后审false保留；详见[S06复盘](../../outputs/stool_assisted_20261021/grant03/S06_round_review.json)。

## S01 / S02 已取得的证据

- 服务器身份/副本：889 项原源码、20 项参考工具逐 SHA 一致；独立普通文件复制、22 个资源链接与回退封存已完成。回执在 `outputs/stool_assisted_20261021/server_bootstrap.json`，不改 Beam 控制基线。
- 编译与真实构建：原 worker 178 阶段；stool 适配为179阶段，原九件 success、资产、目标、顺序和底件 rebase 保留。真实 loader19项、真实 worker及真正 `setup_offset_for_assets` env0/env1 预检通过。服务器运行适配10项、FCL正反例9项通过，均为离线检查。
- S01 原件：URL 资产根被 `abspath` 变成本地路径，背景 USD 加载抛 `FileNotFoundError`，尚未 reset/动作/视频采集。原进程 exit0，原主结果缺失；后审明确拒绝。之后修复对象共用解析器，8项服务器解析测试和4个原URL的HTTP200/SHA传输核查通过，任务目标和资产选择未变。
- S02 原件：实际初始化配置与真实框架/场景映射全签名相符；93网格、3615对初始几何检查通过，九件均解析为唯一 native body，双臂/夹爪/选定场景均列入。背景四个视觉干扰物原配置没有 collider，导出明确保留该跳过原因，仍参加可见网格检查。
- S02 停点：任务step0、native index3，第一条动作尚未下发。独立采集器以机器人容器路径创建 articulation view，backend为空，抛 `AttributeError`。这属于新增工具接口错误，不是抓取或装配物理失败。原主报告false、动作未完成、严格后审false；同样保留原 exit0。
- 两轮均没有产生图像或视频帧；S02已有 recorder空目录，但采集在 raw初始化之后，因此未留下启动三视图，也无失败尾录。59个服务器原件已下载逐SHA验证。每次原后审仅执行一次，旧结果不修改。
- 两轮后候选：从已存在 wrapper 的真实 PhysX `prim_paths` 读取唯一身份，再创建独立 raw视图；拒绝缺失/歧义/错机器人/复用视图。10项服务器纯接口测试通过，未再启动native。此修正不能作为 S02 已通过的证据。

## G02 新增运行证据与边界

- 先验证实际采集器和启动三视图，随后正式运行S03/S04。P01/P02/P04/P05进入native，P03仅shell失败；零动作启动诊断独立计数，不能称装配运行。P05通过独立41个原生刚体成员、双臂完整关节/九件、同钟三视图和真实首策略/适配器/guard只计算验证。启动前检查及原错误保留在G02原件与账本索引中。
- 实际执行链修复包含：独立视图使用真实 PhysX articulation root并即时复制完整关节缓冲；原夹爪/机械臂映射逐DOF核验；stool双UR5e保持连续关节目标，避免最终guard后再重归一化改变SDK分支；安装支撑回调重复object参数错误在离线阶段排除；终态采图异常不覆盖原calculate_metrics报告。上述身份按每轮source/tool binding核对。
- S03已访问底件2的12个阶段、实测开爪/72tick释放及底件rebase；零件3抓取连接后在抬升阶段早期（phase step11）失败。最终预测f0 fixture–part3约2.672µm穿透，同钟原生场景与当次候选动作f1的原冻结关系终点无该碰撞；冻结关系与原生零件有17.77µm残差。原主报告false、完整动作false、严格后审false。
- S04结果：已通过旧step4961/native4964拦截点，随后增加99个SDK批次；在step5060/native5063由实测原生位姿的fixture–零件3可见CAD网格重叠10.027µm触发停止，未提交新批次。原主结果、完整阶段及严格后审均false，零件3抬升尚未完成。该新失败钟完整独立raw缺测，不用native5064的终态图补造；终态三图自身同钟且采图前后原生状态完全不变
- 持件预测修正从同tick实际零件开始，逐样本收敛到原冻结关系的最终预测；原关系/固定连接不写回。位置线性、旋转最短SLERP，采样取原点集与修正点集并集；分辨率取本任务实际最小cartesian increment0.25mm和原joint increment0.035rad，512上限不变。原7mm/0.1rad附着漂移门独立保留，碰撞门/接触对象对不变。两个失败附近full-native状态、24个base2状态与危险反例共33项真实guard/FCL离线回归通过，才进入S04。
- 正式S04复用P05未变的worker/raw/SDK/config域；变更的guard和图像发布比较域先用当前SHA离线检查，S04又执行自身原生采集、几何初始化及三图核验，最终动作由生产guard在SDK提交前检查。P05独立dry-action不冒称S04正式分支也另执行dry-action。
- USD表示比较保留归一化物理矩阵与原残差/失败标志，再独立核实际Gf/UsdOrient对原始float32四元数的转换结果；发布矩阵门仍32eps。164个P05/S03历史实际矩阵行、实际Gf探针和独立后审计算已交叉核对。S03终态采图的旧错误和原报告不修改。
- G02实际34个组成层均封存内容与SHA，旧S02的6个未哈希层仍按原件保留；S02的初始检查不回填为完整身份或非零运动证明。
- 12个连续native tick的抓取资格是本任务几何资格；配置未启用强制指力门，attach时两指有效contact_probe读数均为0，不能称实测受力抓取或无辅助纯物理成功。辅助固定连接及实测安装世界支撑仍参与执行。
- 原被动报告缺项仍列出：18个夹爪模型帧、原报告标记`no_enabled_supported_colliders`的10个对象根、原摘要截断。完整事件流单独对账，严格覆盖结论不自动置true。S04原后审的完整阶段、native终态证书/窗口、wrapper–raw/SDK终态联接及证书钟三视图联接仍false；启动/终态图本身通过不能替代九件终态证书。S03一次凸包候选由同tick完整41体矩阵绑定后，精确CAD间距约1.688mm，凸包重现误差小于1e-15m；这只是该候选的几何见证，不证明native cooked/SDF碰撞域完整。

后续候选、暂停和新运行权限只读[账本](execution_ledger.json)；首次失败、媒体、封存身份和续接索引见[短交接](HANDOFF.md)。

## S01 前基线和适配边界

本节及下方首轮修复包描述改动前的实现与风险；已实现和未验证的边以上方各轮原件、G02证据及账本为准。历史“stool未接入”不表示新增入口尚未实现。

- 服务器已核889项源码/20项参考工具；独立副本与资源链接见 `outputs/stool_assisted_20261021/server_bootstrap.json`，外部运行环境见短交接。完整运行能力仅按上面实际到达边界解释。
- 辅助 canonical compiler 的 `compile_fabrica_canonical_recipe()` 会把 `beam_execution_route=assisted` 分派到 `compile_assisted_beam_recipe()`，后者明确拒绝非 Beam。stool 原配置仍走 generic canonical。需单独的 stool opt-in/任务适配接口，不能通过删除拒绝检查或改名完成迁移。
- Beam 工具不是现成 stool 工具：worker/预检绑定 Beam，visible mesh 检查固定54网格/五件，sweep 工具含服务器基线根和外部 geometry 依赖路径。必须从 stool 九件、双臂/完整夹爪和本场景重建覆盖域；既有 exporter 未覆盖背景场景，不能宣称环境全覆盖。
- 移植先核服务器链接图、资源身份及 Python/Isaac Sim/motion config 等外部依赖；本机已重写的绝对资产链接不可当服务器链接图。普通源码独立复制，共享大资源只读复用，Beam 基线保留。

## 原任务合同

- task_id：`fabrica_stool_circular_ur5e_staged`；assembly：`stool_circular`；UR5e + Robotiq 2F85。逻辑名 `franka_right` 搬底件，`franka_left` 装其余件；名称不代表 Franka 实体。
- 九件：`fabrica_stool_circular_0` 至 `_8`。底件 `2`；顺序 `2 → 3 → 7 → 6 → 4 → 5 → 1 → 0 → 8`。

| 移动件 | 接收件 socket_part | optimizer_hold_part |
|---|---|---|
| 3 | 2 | 2 |
| 7 | 2 | 2 |
| 6 | 3 | 3 |
| 4 | 7 | 7 |
| 5 | 2 | 2 |
| 1 | 2 | 2 |
| 0 | 1 | 1 |
| 8 | 5 | 5 |

- Catalog：`roboassemblybench/assets/Fabrica/canonical_7_bundles/canonical_tasks.json` 中 `tasks.stool_circular`。原始 OBJ 厘米按 `0.01` 转米；pickup 刚体带重力；fixture/optical board 为静态 SDF。分辨率和 margin 是资产配置，不是 native 碰撞质量证明。
- 坐标：位置米，角度弧度，四元数 wxyz。九件资产几何共享装配坐标，编译器为每件生成 world 目标；官方拆卸路径倒序并组合到装配 world frame。socket 关系不等于运行时活动父 frame。
- 底件释放会按实测底件位姿重定位后续目标：compiler `_compile_targets_and_phases()` 的 `rebase_targets`，执行层 `_lock_object()`。必须同时记录原始目标、重定位变换、运行目标，不悄改该合同。
- 原辅助：current-relative fixed joint、接触资格、fixture 锁定、安装 freeze-current lock；实现仍含位姿/速度写入，见下表。不是新的物理约束适配已实现。
- 原成功：九件均到各自运行目标、`require_released=true`、`require_static=true`；姿态门 `0.15 rad`；原速度门 `0.05 m/s`、`2 rad/s`。`_object_velocity_metrics()` 另允许历史位姿稳定替代速度门，默认至少8个 history 样本、漂移不超过 `2 mm/0.05 rad`，不能把调用样本称为 native 连续 tick。
- 位置门由几何 release 包络生成，`release_lock_uses_placement_tolerance=true`。服务器实际 build 必须导出九件最终门及 target 覆盖关系；不先固定为统一 `15 mm`。真实原 worker 已导出确认：底件2约19.209 mm；1/3/5/7约50.510 mm；0/4/6/8约18.028 mm。冻结原配置见本机原件包 `baseline/original_worker_config.json`。
- 原 release advance：已装件静止、`min_steps=72`；不明确要求实测开爪。共享配置 `skip_terminal_park=true`、`skip_terminal_retreat=true`，原合同没有最终双臂回位门。
- 原报告与任务终止分开：stool 原 generic 路线的 `calculate_metrics()` 非 policy 模式允许 `self.success or criteria_passed`；阶段未完成也可能报告条件通过。辅助基线已有更严的 Beam 专用分支，但 stool 未接入。该文件 `execution_route` 也由 Beam 专用标志决定，不能据标签就判断原 stool 的真实辅助类型。原报告必须原样保留。
- 资格/采集配置种子为 `[4906,485,34,12]`；旧脚本 seed412 仅历史线索，不自动作为本次布局。两轮启动前在实际 build 中冻结具体 seed/layout 与配置身份。

## 首轮可分别验证的修复包

这些是已定位的工程/机制差异及证据缺口，不是已证实的动态装配根因。服务器源码核对后在原共用入口最小适配，新增 opt-in 只作用 stool；不复制巨型 Beam 控制器，不盲改速度或 solver。

| 编号/问题和原件 | 修复边界 | 独立验伪与共同验收 |
|---|---|---|
| P0 构建/映射未打通：旧 `runner_task.log` 缺 `ikpy`；当前 environment.yml 已声明3.4.2 | 核实际解释器、模块路径、依赖和 UR5e motion config；使用真实 stool worker 配置 | worker build → 真正 env0/env1 名称改写 → 生产预检；拒缺失/重复/歧义实体；失败不得启动物理。旧依赖错误不能冒称当前服务器缺陷 |
| P1 抓取仍写位姿、清速度：辅助基线 task 的 generic `_attach_object()` / `_create_attachment_joint()`；stool未接入专用辅助分支 | 通过本任务实测双指资格后，只读同 tick hand/body/TCP，创建固定连接并冻结完整变换；不重锚、不写零件状态、不关碰撞 | 捕捉所有 pose/velocity setter 的负例；读回两端锚点、hand→TCP/hand→object/TCP→object 与碰撞状态；以native残差分别核模型/实测漂移。覆盖九件同族调用 |
| P2 安装 lock 为 pin：辅助基线 compiler `_append_release_and_retreat()` 的freeze-current；task generic `_lock_object()` / `_sync_object_states()` | fixture/安装改为物理支撑生命周期；在当前实测位置接住，实测开爪后才移动约束锚点。原运行目标/底件rebase合同保留可追溯性；活动父件若发现需重新适配 | setter负例、支撑/抓取连接切换、实测开爪、原/运行target差异及真实零件到位分别验；不以锚点到位判零件到位。最终保留辅助支撑须报告 |
| P3 双臂/持件最终指令与几何覆盖缺少 stool 证明 | 从九件资产和实际另一臂状态生成对象/碰撞域；完整双方臂、完整夹爪、持件、已装件、fixture/board/环境参与最终限幅及SDK接受后检查 | 错frame、缺对象、危险轨迹负例与可推进正例；记录对象对/阶段豁免、最小间隙与采样范围。有限网格采样不称连续碰撞证明；不删除硬编码数量断言冒充通用化 |
| P4 原成功、动作完成与独立终态证据混淆/缺测 | stool独立采集接线，记录原criteria/阶段完成/实测开爪/native窗口/支撑保留；终止前只读刷新独立raw及三视图同钟身份 | criteria=true但阶段未完成、重复tick、缺帧、过期图像应拒绝严格验收；覆盖稳定窗口所需tick，不插补。原报告覆盖/错误/跳过完整保留 |

第二轮只根据第一轮首失败和包内各项结论形成差异修复；不预填一个没有证据的物理调参包。两轮后复盘；新瓶颈、跨模块或重复错误提前复盘。

## 每轮必需检查与证据

1. 核账本授权、服务器源/工具/资产/配置及回退；冻结构建后的原成功门。新辅助合同中的接触/运动稳定窗口、实测开爪门和支撑对齐速率，依据本任务资产、原接触合同及生产时钟确定并事先记录，不能从 Beam 复制或跑后放宽。
2. 实际 worker build → 框架映射 → 生产预检；按差异分别验危险负例和可推进正例。当前缺失的入口直写未实现，不能用AST/stub/单函数冒充。
3. 全部受影响阶段核双方最终发布、SDK接受/实测反馈、完整夹爪/持件/已装件/环境扫掠；目标接触只豁免明确对象对和阶段。未变域仅在身份和输入域兼容时复用。
4. 一次从头正式验收：确认首动作、证据增长；保留首次失败和必要尾段。记录原主成功、完整阶段完成、严格独立后审三个结果；回位是单列项，未实施不称已回位。
5. 固定机器后审一次；同native时间的第三视角、左腕、右腕、状态/接触/最终目标；保留三路完整原片、SHA和完整解码，制作改动段/首次失败/尾段必要片段。启动前无图或 native 崩溃如实记尾录缺失。
6. 更新本任务账本与短交接；按显式文档清单一次同步并逐SHA，未完成处记pending。没有完整本任务证据就不认定装配成功。
