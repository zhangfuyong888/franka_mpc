# 从单关节到机械臂：实验路线

先按 [README](../README.md) 安装并运行第一课。以下命令都在仓库根目录、激活环境后执行。
每次换参数保留相同目标和时间；指定输出路径时用新名字，避免覆盖。
术语和公式统一见 [控制数学](control_math.md)，调用关系见 [架构](architecture.md)。

## 1. 认识模型、FK 和 IK

```bash
python examples/01_fk_ik.py
```

目标是理解“由关节求手的位置”和“由手的位置反求关节”。脚本从两个 seed 求同一个空间位置，
打印关节终点、FK 结果和与 seed 的距离。预期两者都在 2 mm 位置阈值内，但关节解不要求相同。
默认没有末端朝向约束。练习：改 seed 的一个关节，观察终点姿态及关节解；IK 不会把真实机器人瞬移过去。

## 2. 只学 MPC：单关节理想模型

```bash
python examples/02_single_joint_mpc.py --output outputs/lesson02/base.npz
python examples/02_single_joint_mpc.py --smoothness 3 --output outputs/lesson02/smooth.npz
python examples/02_single_joint_mpc.py --horizon 15 --output outputs/lesson02/short.npz
```

每条命令自动生成同名 `.npz`、`.png` 和 `.json`。图中 2 s 时目标从 0.6 rad 改为 −0.4 rad。
先看加速度是否受 ±2 rad/s² 约束、速度是否受 ±0.7 rad/s 约束，再看掉头时如何减速、过零和反向。
默认末位置误差应小于 0.01 rad，但不同权重的精确结果应以本次摘要为准。
改变一个参数，解释改变的是“看多远”还是“怎样评价运动”。这一步是理想双积分，不是机器人性能实验。
矩阵与代价对应 `MPCProblem`，不是另一套教学求解器。
旧版 `--smoothness 3` 曾因约 6.9e-7 rad/s 的预测速度越界在下一步被拒绝。
现在通过内部速度/加速度 0.1% 裕量及原操作界复核修复，原始状态不裁剪，
该命令已恢复为正常对照实验。平滑权重 3、10 及不同预测步数均有连续 400 步回归测试。

## 3. 只学执行：参考怎样变成力矩

```bash
python examples/03_mit_step.py --feedforward inverse_dynamics --output outputs/lesson03/inverse.npz
python examples/03_mit_step.py --feedforward gravity --output outputs/lesson03/gravity.npz
python examples/03_mit_step.py --feedforward none --output outputs/lesson03/none.npz
```

这里没有 MPC，参考是第 1 关节 0.1 rad 的解析小幅运动，2 s 完成、随后保持，共仿真 5 s。
自动图表比较 J1 实际/参考、所有关节误差和力矩；JSON 记录跟踪 RMS 及饱和。
`--gain-scale 0.5` 只缩放 Kp，Kd 保持原值。观察关闭前馈是否增加误差、改变力矩，
不要预先认定每种设置都应达到同样精度。由此区分“规划给得慢”和“执行跟不上”。

## 4. 接上七轴、IK 和真实闭环

```bash
python -m panda_mpc.demo --config configs/tutorial/01_single_target.yaml --headless --output outputs/lesson04/base
```

先打开 `outputs/lesson04/base/report.md`。默认实验预期末目标 reached，
位置误差 <0.01 m、速度 <0.02 m/s 并保持 0.2 s；实际与参考应基本贴合。
沿 `runner` 阅读目标激活 → IK → MPC → 参考 → MIT → MuJoCo 的调用链。
重点区分 q_goal、q_pred、q_des、q；它们不是四种叫法的同一个变量。

## 5. 固定事件的目标切换与调参对照

```bash
python -m panda_mpc.demo --config configs/tutorial/02_switch.yaml --headless --events configs/switch_events.json --output outputs/lesson05/base
python -m panda_mpc.demo --config configs/tutorial/04_smooth_mpc.yaml --headless --events configs/switch_events.json --output outputs/lesson05/smooth
python -m panda_mpc.analyze --run-dir outputs/lesson05/smooth --against outputs/lesson05/base
```

两次只改变 `delta_a_weight: 0.3 → 3.0`。生成候选运行目录下的
`parameter_comparison.json` 与 `plots/parameter_comparison.png`。
对照程序要求采样时间和目标数组一致，避免把不同任务当成参数效果。
练习：分别用新文件改变 horizon、速度界、加速度界，每次只改一项；比较误差、跟踪偏差、回退和力矩。
到达前被替换是 superseded，不能一律算失败。

## 6. 大范围随机抢占与故障定位

```bash
python -m panda_mpc.demo --config configs/tutorial/03_rapid_analysis.yaml --headless --output outputs/lesson06/rapid
python -m panda_mpc.analyze --run-dir outputs/lesson06/rapid --at 0.86 --joint 2 --window 0.3
```

这是半速、大范围随机目标，默认 0.3～0.9 s 切换一次。可能出现参考回退和短时制动，不要求每个目标到达。
先看 report 中的输入状态拒绝、求解器失败、解复核拒绝，再看 `inspection.json` 与局部图。
旧基线在 0.86 s 的 J2 实测速度略超 0.35 rad/s；增加优化裕量后同一 30 s 实验的 7 次输入拒绝降为 0。
没有降级事件也可用 --at 查看正常控制状态，新配置的实际结果以本次日志为准。
不要把 alpha 理解成新旧目标权重，也不要把 alpha=1 理解成没有制动。

正式重复实验可从第一次的 `events.json` 回放：

```bash
python -m panda_mpc.demo --config configs/tutorial/03_rapid_analysis.yaml --headless --events outputs/lesson06/rapid/events.json --output outputs/lesson06/replay
```

目标表固定后，激活时 IK 仍从当时实测状态重解；数值结果允许小的浮点差异。
目标在循环前经 home IK 预筛，启动时间可能增加，且不保证全过程无接触。

## 7. 外力恢复与证据边界

```bash
python -m panda_mpc.demo --scenario disturbance --duration 12 --headless --output outputs/lesson07/push
python -m panda_mpc.demo --scenario disturbance --duration 12 --headless --no-disturbance --output outputs/lesson07/control
python -m panda_mpc.analyze --run-dir outputs/lesson07/push --compare outputs/lesson07/control
```

比较真实末端偏移、注入前差异和撤去外力后的恢复时间。恢复需要重新满足原位置/速度/驻留判据，
不能只看误差是否下降。MIT 反馈和 MPC 都参与恢复，本项目没有参数自适应律。

完成以上实验后再启用视频及更激烈参数。视频演示配置继续位于 `configs/rapid_switch.yaml`，
不会让基础教学依赖 FFmpeg 或图形上下文。最新结果与尚未验证事项见 [验证记录](validation.md)。
