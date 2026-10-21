# 09：从命令入口跟踪一次目标替换

1. `demo.main` 解析配置/场景/时长/seed/事件回放。`load_config` 从随包 defaults.yaml 加载，覆盖未知键报错。
2. `runner.run` 首先创建独立输出目录和 metadata，即使依赖失败也能落错误。Simulation 创建 plant，初始化 home；InverseKinematics、LinearMPC、NominalDynamics 各有明确职责。
3. GoalManager 保存完整事件表，但控制器仅在 consume 返回新目标时获得它；随机位置/时间使用独立 RNG，预筛记录拒绝原因。
4. 每十物理 tick，在读取实测 state 后 consume。新目标替换 active、清除旧 q_goal 任务意义，但不赋值 state。
5. ik.solve 从 state.q 在独立构型解目标。成功保存 q_goal，否则 rejected。它只代表终点，不能将 IK 速度写 ctrl。
6. ReferenceGenerator.boundary 给上一接受参考的边界加速度；LinearMPC.solve 仍用实际 state.q/state.dq。MPCProblem.vectors 更新 c/l/u，OSQP 求解，残差/约束复核。
7. Robot 对预测关节样点运行独立 FK/contact 检查。合法后取预测第一步端点，构造 Hermite；无解则制动。
8. 每2ms采样同一段的 q/dq/ddq，NominalDynamics 用实测 q/dq 和参考 ddq 生成前馈，构造带最新 goal_id 的 JointCommand。
9. MITAdapter.apply 合成逐关节反馈、模型前馈和总限幅，唯一一次写 ctrl。Disturbance 随后向 plant 外力接口写本步实验力矩，MuJoCo 推进。
10. Telemetry 记录时间一致的步前行、每个MPC tick和事件；finally 保存，analyze 生成真实数据图。GUI 关闭或异常也尝试保存。

运行 `python examples/04_trace_replan.py --output outputs/teaching-switch` 可逐条打印激活 tick 的 IK、QP、alpha 和边界跳变，当前依赖阻塞尚未运行成功。已经验证的代码映射是 `test_qp_assembly_matches_cost_and_dynamics`，不是另一套教学算法。

定位错误从 events 的 goal_id/first_command_at 开始，再看 tick.status、reference_coefficients、tau_raw/saturated 和 state，不要只看末端轨迹图。练习：给每个模块画输入/输出边界，指出哪一层有权改变 plant 状态、哪一层仅修改计算用 MjData。
