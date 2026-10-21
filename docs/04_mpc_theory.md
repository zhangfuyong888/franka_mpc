# 04：加速度级关节空间 MPC

状态 $x=[q;v]\in\mathbb R^{14}$，优化输入 $a\in\mathbb R^7$，目标 $x_g=[q_{goal};0]$。输入为实测 q/dq、本目标固定终点、上一已接受参考边界加速度。加速度变量不是机器人接口。

段内恒加速度积分给出

$$q_{k+1}=q_k+Tv_k+\tfrac12T^2a_k,\quad v_{k+1}=v_k+Ta_k,$$
$$A_d=\begin{bmatrix}I&TI\\0&I\end{bmatrix},\quad B_d=\begin{bmatrix}\tfrac12T^2I\\TI\end{bmatrix}.$$

T=0.02 s、N=25，窗口 0.5 s 不意味着要求 0.5 s 到达。定义 $\|z\|_W^2=z^TWz$：

$$J=\sum_{k=0}^{N-1}[\|q_k-q_g\|_{Q_q}^2+\|v_k\|_{Q_v}^2+\|a_k\|_R^2+\|a_k-a_{k-1}\|_S^2]+\|x_N-x_g\|_{Q_f}^2.$$

默认 $Q_q=100I_7,Q_v=8I_7,R=0.15I_7,S=0.3I_7$，$Q_f=4\operatorname{diag}(Q_q,Q_v)$。状态权重半正定，R 正定。单位缩放相当于 q 权重 cost/rad²、v 权重 cost·s²/rad²、a 权重 cost·s⁴/rad²；不能将它们当物理刚度比较。

a[-1] 来自上一发送段边界的解析加速度，包括桥接和制动修改；不是实测加速度，也不是上次未修改的 MPC 输入。变化代价不构成硬 jerk；jerk 单位应为 rad/s³。

初值等式取真实状态，未来 k=1..N 的 q/v 有界，k=0..N-1 的 a 有界，无松弛变量，终端只加软代价。初值超界或刹不住时拒解，不裁剪测量制造可行性。

只执行第一段，然后用新的实测状态重算，才能抢占目标并反馈修正误差。双积分只是内环较好跟踪时的近似；有限增益 MIT + 非线性 MuJoCo 并不等于这个模型。有限预测窗口不证明递归可行或碰撞安全。

本机用相同 MPCProblem 和真实 OSQP C 库跑了单轴/七轴：0→0.6 rad，2 s 换为 -0.4 rad，8 s 终点误差约 3.45e-9 rad，峰值速度 0.7 rad/s、加速度 2 rad/s²。这些是理想模型结果，不能写成机器人末端到达指标。

![原生 OSQP 理想模型参数对照](evidence/native_osqp/comparison.png)

原始数值见 [summary](evidence/native_osqp/summary.json)。例子：生产依赖安装后 `python examples/02_single_joint_mpc.py`；本机已运行 `python scripts/verify_native_osqp.py`，后者不验证生产 Python OSQP API。

常见错误是把窗口当全程时限、裁剪实测速度、把双积分当真实动力学。练习：N 从 25 改为 15，比较制动与计算耗时，先测量再解释。
