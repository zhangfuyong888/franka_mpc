# 怎样运行实验、怎样判断效果

先按 [README](../README.md) 安装并激活环境，再在项目目录运行。控制算法见 [控制数学](control_math.md)，执行和故障流程见 [架构](architecture.md)。本页将原有实验记录与本次文档整理后重新执行的检查分开。

## 1. 主要实验与命令

| 场景 | 主要验证什么 | 建议仿真时长 |
|---|---|---:|
| `hold` | home 保持、前馈和力矩模型是否正常 | 10 s |
| `single_target` | 0.2 s 激活固定位置后的到达和保持 | 10 s |
| `switch` | 0.2 / 0.7 / 1.3 s 途中换目标，真实状态和参考是否连续 | 12 s |
| `random` | 固定 seed 下随机位置和 2～4 s 间隔的目标管理 | 30 s |
| `rapid_switch` | 每 0.3 s 抢占目标，旧任务不排队 | 10 s |
| `invalid_goal` | 1 s 出现 [2,2,2] m 目标，局部 IK 拒绝后不恢复旧目标而制动 | 8 s |
| `disturbance` | 关节 2 在 4～4.25 s 受到 +6 Nm，造成真实偏移后恢复 | 12 s |

```bash
python -m panda_mpc.demo --scenario hold --headless --duration 10 --seed 42
python -m panda_mpc.demo --scenario single_target --headless --duration 10 --seed 42
python -m panda_mpc.demo --scenario switch --headless --duration 12 --seed 42
python -m panda_mpc.demo --scenario random --headless --duration 30 --seed 42
python -m panda_mpc.demo --scenario rapid_switch --headless --duration 10 --seed 42
python -m panda_mpc.demo --scenario invalid_goal --headless --duration 8 --seed 42
```

`invalid_goal` 预期退出码为 2，不能仅因非零退出码就认为测试失效。若要完整复跑七场景及扰动对照，使用 [acceptance.py](../scripts/acceptance.py)：

```bash
python scripts/acceptance.py
```

它为固定目标场景检查到达判据，为动态目标场景检查正常结束，并验证扰动相对对照的真实偏移和恢复；它不是对所有动态目标逐个到达的证明。

扰动必须与同配置、同目标表的无扰动运行比较。以下输出目录须尚不存在，再次运行时换名字：

```bash
python -m panda_mpc.demo --scenario disturbance --headless --duration 12 --seed 42 --output outputs/push
python -m panda_mpc.demo --scenario disturbance --headless --duration 12 --seed 42 --no-disturbance --output outputs/control
python -m panda_mpc.analyze --run-dir outputs/push --compare outputs/control
```

`--compare` 检查两次配置除 `disturbance_enabled` 外完全一致，且采样时间、目标数组相同。外力每步写入物理数据，撤去后清零；控制器不能读取计划来预先补偿。

现有教学例子可按需要单独运行：[01_fk_ik.py](../examples/01_fk_ik.py) 比较 seed；[02_single_joint_mpc.py](../examples/02_single_joint_mpc.py) 验证理想单轴 MPC；[03_mit_step.py](../examples/03_mit_step.py) 比较前馈/增益；[04_trace_replan.py](../examples/04_trace_replan.py) 输出目标切换的数据流。例子用途不等于本次已经运行了所有例子。

## 2. 状态与指标怎样读

`reached` 表示实际末端位置误差小于 **0.01 m**，末端速度小于 **0.02 m/s**，连续保持 **0.2 s**。这是生命周期中的一个历史事实，之后仍继续控制到该终点；后续扰动可能使当前误差再次增大。`superseded` 表示到达前被新目标替换，或同一 tick 被更新的事件覆盖，不能一律算控制失败。`rejected` 表示本次局部 IK 未通过；失败原因在 `events.json`，不等于全局不可达证明。`active` 表示正在追踪，`pending` 表示尚未激活。

先查看 `summary.json` 的 `execution_completed`，再看每个目标的状态、可用追踪时间与误差。`fixed_goal_acceptance` 只用于 hold、single_target 和 disturbance：要求末目标历史已 reached，且最后样本位置/速度仍满足阈值；其他场景该字段为 null。程序退出 0 与通过到达判据是不同的事。

| 指标 / 文件 | 应怎样理解 |
|---|---|
| `final_error_m`、`plots/error.png` | 相对当前目标的末端欧氏距离；换目标会改变误差基准 |
| `max_ee_speed_m_s`、`joints.png` | 实际速度及实际/参考 q、dq；路过目标但速度很大不算到达 |
| `saturated_steps`、`saturated_joint_samples` | 前者统计有任一关节饱和的物理步数，后者累计每轴饱和样本；结合 `tau_raw` 与 `tau_applied` 看 |
| `qp_failures` | 记录中 MPCResult 未通过接受检查的次数；不包含 IK 拒绝，也不等于所有降级原因 |
| `reference_backtracks`、`bridge_alpha` | alpha 小于 1 的慢环次数，说明候选参考向制动段回退；显式制动可能 alpha=1，所以还要看 mode |
| `degraded_ticks`、`braking_seconds` | 失败事件数及显式制动累计时间；预测接触也可能导致降级 |
| `max_q_reference_jump`、`max_v_reference_jump` | 段切换处的位置/速度参考跳变，检查 C1 接续 |
| `acceleration.png` | 理想 MPC 输入、实际下发参考加速度、真实速度后向差分三者对比 |
| `events.json`、`ticks.json` | 生命周期、监测原因、IK/QP 结果、参考系数及慢环耗时 |

`qp_timing` 是 MPC 包装函数内输入检查、向量更新和求解等时间；`control_tick_timing` 还包含目标处理、同步 IK、预测接触/FK、参考生成及该步内环等，是更接近整个重规划周期的指标。`inner_step_timing` 是每步参考采样、前馈、MIT、监测等时间，不包含物理积分和绘图。P50/P95 是 50%/95% 样本不超过的耗时，`max_ms` 和 `over_budget` 用来观察尾部延迟，不能只看平均数宣称实时。

`simulation_wall_ratio` 用仿真时长除以 runner 的墙钟用时；后者含初始化和循环收尾，未包含之后的分析绘图。GUI 的墙钟等待也会影响它。同步 IK 可能使激活周期明显超过 20 ms。`execution_completed` 主要判断没有异常/监测停止；手动关闭 GUI 可提前结束，仍应同时核对 `duration_s`。

## 3. 仓库原有记录：2026-10-21

以下数值来自本次修改前保留的 [命令及退出码](evidence/closed_loop/commands.json)、[环境与模型摘要](evidence/closed_loop/metadata.json)，**不是本次重新运行整套验收的结果**。这些命令使用 seed=42；表中链接指向原始摘要。

| 场景及证据 | 最终末端误差 m | 饱和步 / QP失败 / 参考回退 | 说明 |
|---|---:|---|---|
| [hold](evidence/closed_loop/hold_summary.json) | 6.42e-15 | 0 / 0 / 0 | home 保持 |
| [single_target](evidence/closed_loop/single_target_summary.json) | 0.000859 | 0 / 0 / 0 | 0.2 s 激活，1.636 s 记录到达 |
| [switch](evidence/closed_loop/switch_summary.json) | 0.001259 | 0 / 0 / 0 | A/B/C 途中替换 |
| [random](evidence/closed_loop/random_summary.json) | 0.010109 | 0 / 0 / 0 | 结束时末目标仍可能在运动 |
| [rapid_switch](evidence/closed_loop/rapid_switch_summary.json) | 0.149497 | 0 / 0 / 0 | 频繁抢占，不要求每个目标到达 |
| [invalid_goal](evidence/closed_loop/invalid_goal_summary.json) | 2.842538 | 0 / 0 / 0 | 相对被拒绝目标的误差；制动，退出 2 |
| [disturbance](evidence/closed_loop/disturbance_summary.json) | 0.000859 | 0 / 0 / 0 | 注入后恢复，另有 [无扰动对照](evidence/closed_loop/control_summary.json) |

原有扰动对照的注入前末端差异为 0，最大实际偏移为 0.006780 m，注入后最大目标误差为 0.007100 m。撤去外力后 0.796 s 重新完成上述位置/速度/驻留条件。恢复包括 MIT 反馈和 MPC 重规划等作用，不能据此单独量化 MPC 的贡献。曲线见 [固定目标误差](evidence/closed_loop/single_target_error.png) 和 [扰动对照](evidence/closed_loop/disturbance_comparison.png)。

原有单目标完整控制 tick 的 P50 / P95 / 最大耗时约为 3.415 / 3.506 / 11.362 ms，20 ms 超预算为 0；invalid_goal 的拒绝 tick 最大约 122 ms。所有结果仅对应记录中的机器与配置。

## 4. 本次重新验证：文档与注释整理

本次只修改文档和 Python 注释/docstring，不修改算法、参数、依赖、接口、执行顺序或实验判据。检查结果及命令保存在 [本次检查记录](evidence/docs_revision/checks.txt) 和 [测试输出](evidence/docs_revision/pytest.txt)。

| 本次实际执行 | 结果 |
|---|---|
| 9 个修改过的 Python 文件与修改前 HEAD 比较，去除 docstring 后的 AST | 全部相同；未改变可执行结构 |
| 其余源码、模型资产、配置、依赖声明、测试、示例及脚本逐字节比较 | 全部未变；已有 26 份证据/日志文件也逐字节保留 |
| `.venv/bin/python -m compileall -q src scripts examples tests` | 通过 |
| `.venv/bin/python scripts/check_docs.py`、`git diff --check` | 相对链接存在性及差异格式检查通过；另检查代码块/公式块配对、Python 片段语法 |
| `.venv/bin/python -m pytest -q -W error` | **22 passed，0 skipped，31.52 s**；没有忽略警告 |
| 控制数学中的单关节两步例子 | 直接执行文档片段，使用生产 `LinearMPC`；预测方程及完整代价复算与表格六位小数一致，见 [数值输出](evidence/docs_revision/numerical_example.json) |

本次测试包含真实 MuJoCo/mink/OSQP 的电机映射与保持、IK 状态隔离、理想 MPC 切换、固定目标 10 s、目标切换及回放 6 s、扰动与无扰动对照各 12 s、无效目标制动 3 s。其余单元测试覆盖代价组装、参考整段边界、事件及失败处理等；其中日志绘图单元测试明确使用合成数据，不作为物理性能证据。没有为了文档或注释新增测试。

本次环境为 Python 3.10.20，主要仿真/求解依赖与 `pyproject.toml` 固定版本一致。所有检查使用既有 `.venv`；未进行全新环境安装。源码字节哈希会因注释改变，但去除 docstring 后的 AST 不变。

核对代码时修正了旧说明容易产生的误解：MPC 有 175 个控制变量但共 539 个 QP 变量；热启动由 OSQP 自动复用结果，没有控制序列移位；参考从上一接受参考边界积分而非直接照搬预测点；默认不约束朝向，且 IK 接受判据没有朝向阈值。实现保持原样。

## 5. 保留的历史资料与尚未验证事项

旧的 00～10 教程及独立符号表已合并到三份详细文档，避免两套主教程并存。原修复经过归档为 [2026-10-21 记录](validation_20261021.md)，初次缺依赖阶段保留在 [最初验证记录](validation_initial.md)；历史测试输出、安装/打包日志、NPZ、JSON、图片和许可证均保留。

[原生 OSQP 离线摘要](evidence/native_osqp/summary.json) 和 [对照图](evidence/native_osqp/comparison.png) 使用已有 CasADi/OSQP C 插件及生产矩阵组装，验证的是理想双积分模型，不是 MuJoCo 物理效果，也不是生产 Python OSQP 接口。CasADi 不是运行本项目的生产依赖。

尚未完成 GUI 自动验收（历史 [X11 失败日志](evidence/gui_startup.log)）、干净环境完整传递依赖锁、多 seed 系统回归、未知负载实验、真实严重数值异常触发验证。本次未重跑整套七场景验收或三种前馈消融。文档的数值例子只验证教学计算，不充当机器人性能结果。无硬件控制、参数自适应或全局避障保证。
