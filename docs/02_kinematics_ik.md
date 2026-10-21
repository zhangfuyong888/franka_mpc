# 02：从空间目标到关节终点

FK 给出 $p=f(q)$，求时间导数得 $\dot p=J_p(q)\dot q$，$J_p\in\mathbb R^{3\times7}$。一阶展开 $f(q+\Delta q)\approx f(q)+J_p\Delta q$ 局部线性化非线性目标。设 world 系误差 $e_p=p_g-f(q)$，教学上的阻尼差分 IK 为

$$\min_v\|J_pv-\kappa_pe_p\|_{W_p}^2+\lambda^2\|v\|^2+\text{姿态/关节软偏好}.$$

阻尼抑制奇异附近的大逆解；冗余由接近 seed 的 PostureTask 处理。虚拟积分 $q^{j+1}=q^j+\Delta t_{IK}v^j$ 只更新独立 Configuration，七铰链允许加法形式，浮动基不能照搬。

实际采用 mink 0.0.13 的 FrameTask/PostureTask，优化变量为位移 Δq。当前使用其公开 build_ik 构造问题，项目内显式转为 CSC 并通过 OSQP1.0.4 求解，再按 solve_ik 相同约定换算： $v=\Delta q/\Delta t_{IK}$。该版本 build_ik 文档字符串有反向换算的笔误，以实际代码为准。项目调用 integrate_inplace，不把 IK 虚拟速度送到执行器。

令 $T_{wb}$ 为当前帧、$T_{wt}$ 为目标帧，FrameTask 误差为 $e=\log(T_{wb}^{-1}T_{wt})$，误差 Jacobian 为 $-J_{\log}(T_{wt}^{-1}T_{wb})J_b$，解 $J_e\Delta q\approx-\mathrm{gain}\,e$。Configuration 将 MuJoCo world 对齐 Jacobian 旋转到 body 坐标。SO(3)/SE(3) 使用对数而非欧拉角减法；教学 world 位置公式不是全部内部实现。

InverseKinematics.solve 从实测 seed 开始。默认位置 cost=1、姿态 cost=0（可配置软偏好）、posture cost=0.001、gain=0.8、lm_damping=1、全局 damping=0.001、ik_dt=0.05 s。mink 的任务 cost 加权残差后平方；全局 damping 直接加入 Hessian，对应教学式 λ²，不应再次平方。

ConfigurationLimit 留 0.08 rad 余量；VelocityLimit 实现 $|\Delta q|\le v_{max}\Delta t_{IK}$。最多 150 次、墙钟预算 1 s，成功条件为位置 <2 mm，并复核有限值、运行关节界和已知接触。姿态误差记录但不作为硬成功条件。

每个目标只解一次，成功后 q_goal 固定。失败有 reason，不追撤销目标。随机点先从 home 预筛、激活再从实测 seed 解；预筛不保证此次局部成功，也不保证路径无碰撞。

例子 `python examples/01_fk_ik.py` 比较两个 seed。依赖已补齐；独立计算状态/可达终点/无效目标测试已通过，并将 SparseConversionWarning 作为错误检查。项目不修改第三方源码，也不通过屏蔽警告处理转换。最新实测见 validation.md。

错误：把 IK 当作有约束时域轨迹；把局部失败称为全局不可达。定位看 reason、seed、误差、迭代及余量。练习：将 orientation_cost 改为 0.02，实测位置成功率和关节分支变化。
