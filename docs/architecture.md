# 项目怎样运行

先读 [README](../README.md) 运行一次固定目标实验。这里沿程序执行顺序解释各层怎样协作；数学细节统一放在 [控制数学](control_math.md)。

## 模块如何分工

闭环图见 README。上层回答“要去哪里、怎样安排运动”，下层回答“这一刻施加多大力矩”。它们通过明确的数据对象交接，避免把预测状态当成真实机器人。

| 模块与入口 | 输入 → 输出 | 负责什么 |
|---|---|---|
| [demo.main](../src/panda_mpc/demo.py)、[config.load_config](../src/panda_mpc/config.py) | 命令行、YAML → 配置、退出码 | 加载随包默认值、覆盖及校验，调用 `runner.run` |
| [runner.run](../src/panda_mpc/runner.py) | 配置、输出路径、事件文件 → 实验文件 | 协调所有模块及执行顺序 |
| [goals.make_events / GoalManager](../src/panda_mpc/goals.py) | 场景或事件表、仿真时间 → 当前目标记录 | 生成事件、只激活最新目标、记录到达和替换 |
| [ik.InverseKinematics.solve](../src/panda_mpc/ik.py) | 世界位置、实测关节 seed → `IKResult.q_goal` | 独立计算局部关节终点 |
| [mpc.LinearMPC.solve](../src/panda_mpc/mpc.py) | 实测 q/dq、q_goal、上一参考加速度 → 预测状态与加速度序列 | 组装、更新并求解带运动边界的 QP |
| [reference.ReferenceGenerator](../src/panda_mpc/reference.py) | 上一参考边界、第一优化加速度 → 可按时间采样的参考段 | 保持位置/速度连续，检查整段边界，必要时制动 |
| [dynamics.NominalDynamics.feedforward](../src/panda_mpc/dynamics.py) | 实测 q/dq、参考加速度 → 前馈力矩 | 在独立名义模型计算惯性、偏置及被动力项 |
| [mit.MITAdapter.apply](../src/panda_mpc/mit.py) | `JointCommand`、实测状态 → 合成并限幅的力矩 | 唯一的控制 `ctrl` 写入点 |
| [sim.Simulation](../src/panda_mpc/sim.py)、[robot.Robot](../src/panda_mpc/robot.py) | 力矩 → 下一物理状态；候选 q → FK/接触查询 | 推进物理、按名称索引关节、隔离计算查询 |
| [monitor.Monitor](../src/panda_mpc/monitor.py)、[disturbances.Disturbance](../src/panda_mpc/disturbances.py) | 实测状态/执行结果；实验计划 → 事件或外力 | 监测异常、注入并清除物理外力 |
| [diagnostics](../src/panda_mpc/diagnostics.py)、[doctor](../src/panda_mpc/doctor.py) | 实验文件或环境 → 报告、时间定位、参数对照、自检结果 | 离线教学诊断与安装自检，不参与控制 |
| [telemetry.Telemetry](../src/panda_mpc/telemetry.py)、[analyze.analyze](../src/panda_mpc/analyze.py) | 状态/命令/事件 → 数据、摘要与图片 | 保存事实、离线评估 |
| [visualizer.Visualizer](../src/panda_mpc/visualizer.py) | 实测末端、目标、预测 FK → 窗口 | 约每 40 ms 仿真时间刷新，关闭窗口可结束循环 |
| [video.VideoRecorder](../src/panda_mpc/video.py)、[rendering.SceneOverlay](../src/panda_mpc/rendering.py) | 步前状态、相机、目标与轨迹 → MP4 帧 | 可选离屏渲染、共享轨迹装饰、按仿真时间流式编码 |

跨层数据定义在 [types.py](../src/panda_mpc/types.py)：`RobotState` 是测量快照，`GoalRecord` 保存生命周期，`MPCResult` 保存预测与求解质量，`JointCommand` 包含位置/速度参考、增益、前馈、时间戳和有效期。量的维度与单位统一见控制数学的符号表。

## 启动到第一个控制周期

`python -m panda_mpc.demo` 先由 `main()` 解析参数。`load_config()` 从包内 [defaults.yaml](../src/panda_mpc/defaults.yaml) 读取默认值，再覆盖配置文件及命令行；[configs/default.yaml](../configs/default.yaml) 是可供用户修改的同值样例。周期必须是物理步长的整数倍。

`runner.run()` 首先创建一个新的输出目录并保存配置和环境信息。随后初始化 MuJoCo 模型，在 `home` 关键帧设置一次物理状态，创建 IK、MPC、名义动力学、MIT、目标管理器、监测器及参考生成器。参考第一次从实际位置/速度开始。

随机场景在循环前生成目标表：位置与时间使用独立随机数流，从工作空间盒内抽样，并从 home 做局部 IK 预筛。运行时只有到期目标进入控制器；目标真正激活时，仍会从当时的实测构型重新求 IK。预筛通过不保证从任意后续 seed 都求解成功，也不是未来轨迹预瞄。普通目标表和记录的 `events.json` 均可用于回放。

`random` 使用 `random_box/random_interval`；`rapid_switch` 使用独立的
`rapid_switch_box/rapid_switch_interval`，默认空间更大、间隔更短。两者都从 0.2 s
开始逐个累加随机间隔；与是否到达无关。每个目标最多抽样 100 次，未能找到通过预筛的
位置则明确报错。`events.json` 的 sampling 记录本次空间范围、间隔范围、接受数与拒采原因。

模型来自随包的 [Panda 资产](../src/panda_mpc/assets/panda/README.md)，保留惯量和几何，固定双指，使用七个直接力矩电机；`nq=nv=nu=7`。`robot.load_model()` 核对电机语义，`Robot` 核对关节地址、传动顺序及留有裕量的操作边界。派生方法在 [prepare_model.py](../scripts/prepare_model.py)，来源与许可见 [第三方声明](../THIRD_PARTY_NOTICES.md)。

## 2 ms 与 20 ms 怎样协作

物理步长是 2 ms，即每秒 500 步；每十步运行一次 20 ms MPC，即每秒 50 次。使用整数 `tick % stride` 调度，避免浮点取模误差。IK 在有新目标的慢环周期同步执行。20 ms 是仿真中的重规划间隔，不是求解必定在 20 ms 墙钟时间内完成的保证。

下面是**根据 runner.py 简化的流程**，省略参数、日志字段和异常包装，不是源码原文：

```python
for tick in range(steps):
    state = sim.begin_step()             # mj_step1 后读取步前真实状态
    monitor.inspect_state(state, data)
    if monitor.stopped:
        break
    if tick % stride == 0:
        new = goals.consume(state.t)
        if new:
            # 只更新本次目标记录；失败标记 rejected
            solve_and_store_ik(new, state.q)
        qs, vs, previous_a = ref.boundary(state.t)
        if active_goal_has_q_goal():
            result = mpc.solve(state.q, state.dq, q_goal, previous_a)
            # 还要检查结果质量、离散预测接触，并累计控制失败
        if valid_prediction():
            ref.advance(state.t, result.a_pred[0], goal_id)
        else:
            ref.update(state.t, qs, zeros, goal_id, braking=True)
    if monitor.stopped:
        break
    qr, vr, ar = ref.segment.sample(state.t)
    ff = nominal.feedforward(state.q, state.dq, ar, mode)
    command = make_joint_command(qr, vr, ff, segment_end)
    actuation = adapter.apply(data, command, state)
    monitor.inspect_actuation(state.t, actuation, goal_id)
    external = disturbance.apply(data, state.t)
    goals.observe(state)
    record_pre_step(state, command, actuation, external)
    sim.end_step()                       # mj_step2 推进 2 ms
    check_simulation_time_and_update_viewer()
```

每行日志的时间 `t` 对应物理推进前的测量和本步命令。`actual_acceleration` 用相邻真实速度做后向差分，首行不可用；它不是 `a_pred[0]`。`prediction_error` 比较上一预测下一点与当前实测状态。慢环之间沿用同一参考段，但每 2 ms 都重新采样和计算反馈/前馈。

## 目标替换时保留什么

同一慢环若有多个事件到期，只激活计划时间最新的一个；同时间按输入顺序取最后一条，其余记录为 `superseded`，不逐个求 IK。原先仍为 `active` 的目标变成被替换；已经 `reached` 的历史事实保留，并记录替换时间。

新目标成功求 IK 后，固定其 `q_goal`，后续每 20 ms 对这个终点重新做 MPC，不重复求 IK。更新的是目标记录、优化目标和当前计划；实测 q/dq、上一参考段边界以及 OSQP 的内部热启动状态都保留。到达后继续保持这个终点，直到被替换。到达使用位置、速度及连续驻留条件，见 [验证](validation.md)。

## 为什么有多套计算状态

真实 `sim.data` 保存物理状态，初始化后由 MuJoCo 动力学推进，不能把 IK 解直接赋给它。否则机械臂会瞬移，既没有力矩过程，也无法验证扰动恢复。

mink 的 `Configuration` 拥有独立计算数据，用于虚拟 IK 迭代；`Robot.scratch` 用于 FK（正运动学，即由关节角求末端位置）和预测接触查询。这两者共享只读用途的物理模型，但不共享物理状态。`NominalDynamics` 进一步使用独立模型和数据，其输入仅是实测 q/dq 和参考加速度，不读取物理外力、接触约束力或扰动计划。

## 失败后怎样处理

| 情况 | 当前实现的处理 |
|---|---|
| 新目标 IK 失败 | 最新目标标记 `rejected`，不恢复旧 q_goal，从上一参考边界制动；局部失败不证明全局不可达 |
| MPC 被拒绝或预测构型接触 | 记录 `degraded`，本周期制动；后续周期仍可尝试当前有效 q_goal |
| 连续控制失败 | 计数达到默认 30 次停止；成功且无预测接触的 MPC 周期清零计数 |
| 候选参考超界 | 逐步混合向制动曲线回退；连制动段也不合法则抛出 `ReferenceFailure` 并停止 |
| 命令过期或时间戳来自未来 | MIT 仅输出限幅后的阻尼力矩，记录 `emergency_damping` |
| 实际非有限状态、越物理关节界、不允许接触、MuJoCo warning 或时间跳变 | 设置停止标志并留存原因 |
| 持续力矩饱和 | 任一关节连续饱和达到默认 0.5 s，设置停止标志 |

停止标志并不等于随时中断当前指令：步前和参考更新后的检查可在 `step2` 前退出；力矩饱和在本步执行检查时才发现，当前循环仍会记录并推进这一物理步，下一轮退出。它是可观测的实验停止机制，不是硬件急停保证。预测接触检查只覆盖离散构型，不进行连续扫掠或主动绕障搜索。

## 数据、评估和窗口各做什么

`Telemetry` 每 2 ms 保存实际/参考/力矩，每 20 ms 保存 QP、IK、参考系数和耗时，另保存目标及监测事件。循环结束或捕获异常后在 `finally` 中落盘；`analyze` 随后读取数据，用无窗口绘图库生成摘要和图，并由 `diagnostics` 写出易读的 `report.md`。没有物理数据时只报告失败，不画虚构轨迹。

可视化显示红色目标、绿色实际末端历史与蓝色预测关节点的 FK 位置。实际轨迹按目标编号分段，
只保留最近 4 条已结束的历史轨迹（较淡绿色），另显示当前轨迹；更早的轨迹自动移出窗口，
日志仍保留完整数据。可视化只负责展示和适当墙钟节奏控制，不参与优化。`--headless` 不导入 viewer。
耗时字段的统计范围和实验判据见 [validation.md](validation.md)。

开启 `record_video` 后，循环在保存步前日志后、`step2` 前调用 `VideoRecorder.capture`，
以同一物理时刻的几何数据渲染。整数帧号除以 FPS 决定下一帧时间，不使用墙钟定时；
RGB 帧通过管道交给独立 FFmpeg 进程编码 H.264/yuv420p MP4，不把整段视频存入内存。
启用 `video_webm` 时结束后额外转为 VP9 WebM，方便缺少 H.264 插件的播放器观看。录制与窗口共用 `SceneOverlay` 的四段历史及当前轨迹，
但使用独立相机，窗口拖动不改变视频构图。结束时 `finally` 关闭编码器和渲染器，再保存实验文件。
普通不录制的 headless 路径不导入视频库或创建 Renderer。录制开关与命令见 [README](../README.md)。

## 教学与复现入口

`runner._activate_latest_goal()` 集中处理最新目标激活和 IK 结果，主循环继续明确协调
慢环规划与快环执行；没有新增隐藏的状态更新或替换控制模型。失败结果新增 `failure_kind`，
仅用于诊断分类，旧的求解接受条件保留。

`metadata.json` 从 schema_version=2 开始保存 Git 提交号和 dirty 标志；`summary.json`
增加运行满时长标志及 MPC 拒绝分类。历史字段仍保留，诊断模块能从旧 status 推断原因。

入门配置位于 `configs/tutorial/`，不依赖录制；原演示配置仍在 `configs/rapid_switch.yaml`。
`doctor` 只做环境检查，可选渲染检查在子进程进行。可运行实验与练习见 [实验路线](lessons.md)。
