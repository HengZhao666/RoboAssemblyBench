# 新 Fabrica 任务：辅助修复入口

2026-10-21，W06.0。供新对话把 Beam 积累的方法用于本项目其他 Fabrica 装配任务。本次只整理流程/经验及性能分析；没有选择新任务、建立新运行授权、修改控制代码或启动仿真。Beam 的状态/权限仍只读[辅助账本](outputs/beam_assisted_20261015/repair_batch_D_20261016.json)。

## 新对话最少读什么

1. 本页及 [fabrica-assisted-repair](skills/fabrica-assisted-repair/SKILL.md)。新任务先核用户指定的实际 task/config；未给名称则只读查找候选并询问，不能默认为 Beam。
2. 已存在的该任务交接/账本；没有则按[最小模板](skills/fabrica-assisted-repair/references/task-handoff-template.md)补任务合同、源身份、短问题清单和授权边界。只填已核实内容，未知项明列，不要求一次写全所有历史。
3. 执行时读[共用修复闭环](skills/simulation-repair-loop/SKILL.md)。只有迁移具体机制时展开[源码映射和适配边界](skills/fabrica-assisted-repair/references/implementation-map.md)，不要默认读整个 Beam 交接/发现史。

## 已有经验可以带走什么

固定连接、接触门和安装锁定在 Fabrica 原版中已经存在；Beam 的改进是让辅助过程更可控、避免掩盖碰撞，并把控制缺陷和证据缺口分开。可迁移的是：合格接触后冻结实测腕—件关系、完整位姿跨阶段连续、双方/持件/已装件的最终命令检查、实测位置世界支撑后张夹和缓落，以及真实构建链和同钟终态证据。

当前实现仍含 Beam 专用资产、名称、数量、路径和数值；尚未抽成可无改动运行所有 Fabrica 的通用库。新任务先适配几何、抓取、顺序、另一臂动作和原成功合同，再按差异迁移共用代码，不能照抄 Beam 的64迭代、间隙/稳定阈值或五件成功条件。实现位置与逐项验证层级只维护在上面源码映射中。

## 一轮如何多修、少返工

先列本轮已知问题及同族调用点，形成一个修复包；已定位、可分别验证且无冲突的问题一起修，一次正式验收。程序接口错误用实际 worker 构建/框架名称转换/预检先拦截；控制错误用原失败输入和危险/推进反例验证。涉及强耦合未知物理原因才拆短对照。未变域复用已有证据，机器后审、完整解码和文档同步各一次；不要让重复审查、额外报告或无依赖文档拖住已授权启动。详细规则只在共用技能维护。

## 直接用于新对话

把下面任务名和授权范围改成自己的要求后发送。若只想先诊断，保留“暂不启动仿真”；希望执行时写明本次是否含短物理、完整验收和轮数/暂停要求。

> 我要修复本项目 Fabrica 的【实际任务名/配置路径】，使用与 Beam 相同类别的物理辅助。先读根 FABRICA_ASSISTED_REPAIR_HANDOFF.md，使用 $fabrica-assisted-repair 和 $simulation-repair-loop，再核本任务权威源码、配置和已有证据。不要加载 Beam 全部历史，不复制其数值、成功结论或运行授权。先整理本任务的短交接、原成功条件和可分别验证的多问题修复包。本次授权范围：【例如先诊断并修改代码，暂不启动仿真；或完成一轮含完整验收和完整视频下载，随后暂停】。只对缺失且影响执行的信息询问。

继续 Beam 本身仍走 [BEAM_REPAIR_HANDOFF.md](BEAM_REPAIR_HANDOFF.md)。Beam 的减步/提速证据和未实施建议在[性能分析](beam_routes/assisted/PERFORMANCE.md)。文档/技能版本与同步收据见 [registry](beam_routes/registry.json) 和[显式清单](beam_routes/docs_manifest.json)。
