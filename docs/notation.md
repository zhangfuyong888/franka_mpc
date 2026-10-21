# 符号、单位与源码映射

| 符号 | 维度 | 单位/意义 | 代码 |
|---|---|---|---|
| q,v |7|rad、rad/s，实测 | RobotState.q/dq |
| p、p_g |3|world m | p_ee、GoalEvent.position_world |
| Jp |3×7|m/rad，world对齐 | Robot.state 局部jac |
| x |14|[q;v] | MPCProblem 的状态块 |
| a |7|rad/s²，优化量 | MPCResult.a_pred |
| T、N |标量|.02s、25步 | period、horizon |
| Ad、Bd |14×14、14×7|离散双积分 | MPCProblem.Ad/Bd |
| Qq、Qv、R、S |7×7|经单位缩放的代价权重 | q/v/a/delta_a_weight |
| Qf |14×14|终端软代价 | terminal_scale |
| z |539默认|全预测状态与输入 | MPCProblem.nz |
| P |539×539|QP Hessian | MPCProblem.P |
| c |539|QP线性项 | vectors 的 linear |
| G |889×539默认|初值/动力学/未来界 | MPCProblem.G |
| q_ref/v_ref/a_ref |7|rad、rad/s、rad/s² | q_des/dq_des/ddq_des |
| Kp/Kd |7元素对角|Nm/rad、Nm·s/rad | JointCommand.kp/kd |
| tau_ff/raw/applied |7|Nm | 前馈/合成/限幅 |
| M |7×7|名义惯量 | mj_mulM 隐式使用 |
| h |7|Nm，bias-passive | NominalDynamics.feedforward |
| dtIK |标量|虚拟积分秒，不是物理步 | ik_dt |

记录的 actual_acceleration 来自真实速度后向差分，与优化输入不同。原生 OSQP 离线理想数据只有合成关节状态，不含真实机械臂状态或末端米级误差。
