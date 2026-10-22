# 核心算法与数学实现

本文沿“任务目标 → IK → MPC → 连续参考 → MIT 执行”阅读。模块与主循环见 [架构](architecture.md)，实验和证据见 [验证](validation.md)。所有默认数值以 [defaults.yaml](../src/panda_mpc/defaults.yaml) 为准。

## 1. 从运动途中换目标开始

假设机械臂正在追踪目标 A，此时手还在移动，目标 B 出现了。程序不会把机器人直接放到 B，也不会先停稳再开始：

1. IK 用当前关节构型作起点，求“手到 B 时，七个关节可以是什么角度”，得到新的关节终点。
2. MPC 用**当前实际位置和速度**预测未来，优化朝新终点运动的加速度序列。
3. 参考生成器取第一组加速度，从上一段参考的位置和速度接续，生成这一段要下发的命令。
4. MIT 执行层根据参考与实测的偏差，加上模型前馈，输出限幅力矩。
5. MuJoCo 推进物理状态。下一周期重新测量和求解，修正预测与实际的差异。

这里的“终点”“预测”“命令”和“测量”是四种不同的量。`q_goal` 是 IK 的终点，不是这一刻直接下发的位置；`q_pred` 是 MPC 认为将发生的运动，不保证等于实际运动；`q_des` 才是交给执行层的位置参考。

| 符号 / 代码名 | 默认维度 | 单位与含义 |
|---|---:|---|
| $p_g$ / `position_world` | 3 | m，世界坐标系末端目标 |
| $p=f(q)$ / `p_ee` | 3 | m，FK 算出的当前末端位置 |
| $J_p$ | 3×7 | m/rad，关节小变化到末端位置小变化的映射 |
| $q,v$ / `RobotState.q,dq` | 各 7 | rad、rad/s，实际测量 |
| $q_g$ / `q_goal` | 7 | rad，当前目标的固定关节终点 |
| $q_k,v_k$ / `q_pred,dq_pred` | 各 26×7 | rad、rad/s，预测状态序列，含初始点 |
| $a_k$ / `a_pred` | 25×7 | rad/s²，预测加速度序列；$a_0$ 对应 `a_pred[0]` |
| $x_k=[q_k;v_k]$，$x_g=[q_g;0]$ | 各 14 | 混合 rad 与 rad/s，预测状态和目标状态 |
| $T,N$ / `period,horizon` | 标量 | 0.02 s、25 步；预测窗口 0.5 s |
| $dt,dt_{IK}$ / `dt,ik_dt` | 标量 | 0.002 s 物理步长、0.05 s IK 虚拟步长 |
| $q_s,v_s$ / `boundary_q,boundary_v` | 各 7 | rad、rad/s，上一接受参考在接缝的边界 |
| $q_{ref},v_{ref},a_{ref}$ / `q_des,dq_des,ddq_des` | 各 7 | rad、rad/s、rad/s²，真正下发的参考及导数 |
| $a_{prev}$ / `previous_a` | 7 | rad/s²，上一接受参考的边界加速度 |
| $A,B$ / `Ad,Bd` | 14×14、14×7 | 离散运动模型矩阵，各块含相应时间系数 |
| $Q_q,Q_v,R,S$ | 各 7×7 | 位置、速度、加速度、加速度变化的代价权重；按相应单位缩放 |
| $Q,Q_f$ | 各 14×14 | 状态代价和终端状态代价权重 |
| $z$ | 539 | 混合状态与加速度单位，QP 的全部决策变量 |
| $P,c,G,l,u$ | 539×539、539、889×539、889、889 | QP 的二次项、线性项、约束矩阵、下界、上界；各行/块单位依其变量 |
| $K_p,K_d$ / `kp,kd` | 各 7，等效对角矩阵 | Nm/rad、Nm·s/rad，位置刚度和速度阻尼 |
| $M(q)$ | 7×7 | 等效关节惯量矩阵，使 $M a$ 的单位为 Nm |
| $\tau_{ff},\tau_{raw},\tau_{applied}$ | 各 7 | Nm，前馈、合成力矩、实际写入电机的限幅力矩 |

## 2. IK：知道手要到哪里，怎样求七个关节角

**逆运动学（IK）**就是从末端目标反求关节构型。它只回答终点问题，不安排何时到达。代码入口为 [InverseKinematics.solve](../src/panda_mpc/ik.py)：输入世界位置和七维 `q_seed`，输出带成功标志、误差、耗时及 `q_goal` 的 `IKResult`。

### 2.1 从正运动学走一小步

正运动学 FK 是已知关节角求末端位置：

$$
p=f(q),\qquad f(q+\Delta q)\approx f(q)+J_p(q)\Delta q.
$$

**Jacobian（雅可比矩阵）**描述每个关节转动一点点会让手朝哪里移动、移动多少。在当前位置附近，可以尝试让 $J_p\Delta q$ 接近目标误差 $p_g-f(q)$。但这个近似只在局部有效，所以差分 IK 要反复“计算小步 → 更新计算构型 → 重新求误差和 Jacobian”。

一个便于理解的**位置任务教学近似**是：

$$
\min_{\Delta q}
\|J_p\Delta q-\gamma(p_g-f(q))\|_{W_p}^2
+\lambda\|\Delta q\|^2
+\|q+\Delta q-q_{seed}\|_{W_s}^2,
$$

同时给小步施加关节位置及虚拟速度限制。这里 $\|y\|_W^2=y^TWy$。阻尼项让接近奇异构型时的小步不要过大；限位避免迭代走出允许范围；构型偏好让解尽量靠近 seed。七个关节仅满足三维位置通常有多种选择，因此 seed 和偏好会影响得到哪一个局部解。关节构型偏好与末端朝向偏好不是同一件事。

### 2.2 实际 mink 任务与 QP

当前使用 mink 0.0.13，底层模型和运动学来自 MuJoCo。`FrameTask` 指向 `ee_site`，位置 cost 为 1、gain 为 0.8、任务 LM 阻尼为 1；`PostureTask` 的目标设为本次 seed，默认 cost 为 0.001。全局 `ik_damping` 为 0.001。

上面的教学公式不是对 mink 内部公式的逐字复制：实际 `FrameTask` 用局部坐标中的六维位姿误差，以变换的对数表示位置与转动差异，Jacobian 也包含符号及对数映射修正。这里的 SE(3) 只是“同时表示位置和朝向的刚体变换”；无需展开李群推导就能理解调用过程。mink 的任务 cost 先乘到误差和 Jacobian 上，再构造二次项，因此不能把它的 cost 数值直接当作前面 $W$ 的矩阵元素。

`mink.build_ik(...)` 汇总 `FrameTask`、`PostureTask`、`ConfigurationLimit` 和 `VelocityLimit`，返回 `qpsolvers.Problem`，变量是关节位移 $\Delta q$：

$$
\min_{\Delta q}\ \tfrac12\Delta q^TP_{IK}\Delta q+c_{IK}^T\Delta q,
\qquad G_{IK}\Delta q\le h_{IK}.
$$

当前项目没有通过 `qpsolvers.solve_qp` 求这个问题，而是显式把 mink 生成的密集矩阵转为 SciPy CSC 稀疏格式，取 $P$ 的上三角后直接调用 OSQP。这样避免后端隐式转换的警告。OSQP 接口的 `q` 参数是线性系数，绝不是机器人关节角。

以下是 [ik.py](../src/panda_mpc/ik.py) 中关键语句，省略了约束和状态检查：

```python
problem = mink.build_ik(self.configuration, [self.task, self.posture],
                       dt=self.c['ik_dt'], damping=self.c['ik_damping'],
                       limits=self.limits)
P = sparse.triu(sparse.csc_matrix(problem.P), format='csc')
# solver.setup(... q=problem.q, A=G, l=-inf, u=h ...)
result = solver.solve(raise_error=False)
vel = result.x / self.c['ik_dt']
self.configuration.integrate_inplace(vel, self.c['ik_dt'])
```

求得的 `result.x` 是 $\Delta q$，除以 $dt_{IK}$ 得到速度，再积分这个虚拟时长，构型恰好走一步 $\Delta q$。`ik_dt=0.05` 是数值迭代的虚拟步长，也参与速度限位换算，不是物理机器人真的运动了 50 ms。`integrate_inplace` 只改 mink 自己的计算数据，不改 `sim.data`。

这里每个局部 QP 使用 `eps_abs=eps_rel=1e-7`、`max_iter=10000`，仅接受 OSQP 状态 `solved` 且有解；随后检查速度有限。外层最多 150 次更新，并在迭代间检查 1 s 墙钟预算；这不是能中途抢占单次求解的硬超时。

### 2.3 什么时候认为 IK 成功

seed 必须在操作限位内。迭代到末端世界位置误差小于 0.002 m 后，还要检查关节角有限、在留有 0.08 rad 裕量的操作范围内（终点复核容许 $10^{-7}$ 数值误差），并通过终点接触检查。允许固定基座与地面接触；其他穿透深于 0.001 m 的接触会被拒绝。

默认 `orientation_cost=0`，不要求末端朝向。代码仍给 `FrameTask` 设置 seed 朝向（或显式传入的朝向偏好）并记录朝向误差，但**成功判据只检查位置误差，没有朝向阈值**；增大朝向 cost 也不会新增这个判据。

激活目标时从实测 q 求一次 IK，成功后的 `q_goal` 在目标存续期间固定。随机目标的启动预筛是额外的 IK 调用，见 [架构](architecture.md)。局部求解失败可能来自 seed、限位、接触、迭代或时间预算，不能直接推断目标在全局上不可达。

## 3. MPC：从终点安排未来运动

**模型预测控制（MPC）**每次用模型预测未来，优化一整段运动，只采用第一段，再根据实测状态重新计算。它不是随机生成很多轨迹挑一条；本项目把问题写成二次规划（QP，即二次代价配上线性约束），交给 OSQP 利用矩阵结构做数值迭代。

入口是 [LinearMPC.solve](../src/panda_mpc/mpc.py)：输入实测 q/dq、`q_goal` 和上一接受参考的边界加速度；输出未来状态、加速度序列及求解质量。`MPCProblem` 负责矩阵构造。

### 3.1 为什么优化加速度

设预测状态和控制量为

$$
x_k=\begin{bmatrix}q_k\\v_k\end{bmatrix}\in\mathbb R^{14},
\qquad u_k=a_k\in\mathbb R^7.
$$

这里 $u_k$ 指控制量，下文 QP 的无下标 $u$ 指约束上界。加速度积分后自然得到位置和速度，所以内部优化加速度，仍可对执行层提供位置、速度参考。

每个预测步中假设加速度恒定：

$$
q_{k+1}=q_k+Tv_k+\tfrac12T^2a_k,\qquad
v_{k+1}=v_k+Ta_k.
$$

写成矩阵就是

$$
x_{k+1}=Ax_k+Ba_k,\quad
A=\begin{bmatrix}I_7&TI_7\\0&I_7\end{bmatrix}\in\mathbb R^{14\times14},\quad
B=\begin{bmatrix}\tfrac12T^2I_7\\TI_7\end{bmatrix}\in\mathbb R^{14\times7}.
$$

它对应 `MPCProblem.__init__` 的两行：

```python
self.Ad = sp.bmat([[eye, T*eye], [None, eye]], format='csc')
self.Bd = sp.vstack([0.5*T*T*eye, T*eye], format='csc')
```

这是上层运动预测模型，没有质量矩阵、重力或力矩变量。完整机械臂动力学由 MuJoCo 物理仿真和名义前馈处理，因此本项目是关节空间线性 MPC，不是完整动力学的非线性 MPC。

### 3.2 25 步到底优化多少变量

25 步、每步七轴，需要 $25\times7=175$ 个加速度变量。当前实现**还显式保留所有状态**，没有消去 $x_0$ 到 $x_{25}$：

$$
z=[x_0;\ldots;x_N;a_0;\ldots;a_{N-1}],\qquad
\dim z=26\times14+25\times7=539.
$$

其中 `ns=364` 是状态部分长度，`nz=539` 是 QP 总变量数。175 只是控制序列变量数。

### 3.3 代价函数在衡量什么

令 $x_g=[q_g;0]$，$Q=\operatorname{diag}(Q_q,Q_v)$，且 $a_{-1}=a_{prev}$。与源码一致的完整代价为

$$
J=\sum_{k=0}^{N-1}\left[
\|q_k-q_g\|_{Q_q}^2+\|v_k\|_{Q_v}^2+
\|a_k\|_R^2+\|a_k-a_{k-1}\|_S^2
\right]+\|x_N-x_g\|_{Q_f}^2.
$$

| 项 | 默认权重 | 想解决的问题；增大权重的一般倾向 |
|---|---|---|
| 位置误差 | $Q_q=100I_7$ | 希望靠近终点；相对更重视到达，可能更积极地运动 |
| 速度 | $Q_v=8I_7$ | 希望到终点时停稳；通常更抑制高速运动 |
| 加速度 | $R=0.15I_7$ | 避免过大的运动输入；通常更温和，但可能更慢 |
| 相邻加速度差 | $S=0.3I_7$ | 减少输入突变；更倾向于平缓改变加速度 |
| 终端状态 | $Q_f=4Q$ | 更重视预测窗口末端的状态，而非只看眼前 |

这些是相对权衡，不是无条件的性能保证；边界和各项竞争会改变结果。代码也计入 $x_0$ 的状态代价，但由于 $x_0$ 已固定，这部分对当前最优输入没有影响。

$a_{prev}$ 取上一接受参考段在当前边界的解析加速度，可能已经受回退影响；它不是实测速度差分，也不一定等于上一次未经修改的理想 MPC 输入。

最优代价不必为零：初始误差、到达所需的非零输入、限位和软终端权重都可能让最优值大于零。$NT=0.5$ s 是“这次看多远”，不是“必须在半秒内到达”。末端没有硬等式 $x_N=x_g$，下一周期还会继续向前预测。

### 3.4 哪些限制真正进入 QP

$$
x_0=[q_{measured};v_{measured}],\qquad x_{k+1}=Ax_k+Ba_k,
$$

$$
q_{min}\le q_k\le q_{max},\quad -v_{max}\le v_k\le v_{max}
\quad(k=1,\ldots,N),
$$

$$
-a_{max}\le a_k\le a_{max}\quad(k=0,\ldots,N-1).
$$

位置界来自模型关节范围两侧各缩进 0.08 rad；速度界为前四轴 0.7、后三轴 0.9 rad/s；加速度界为前四轴 2、后三轴 3 rad/s²。实测状态在调用求解器前还会检查操作位置与速度界，只容许 $10^{-10}$ 的浮点舍入误差，实际值原样进入初值等式。若把超界测量裁剪到界内，就等于让优化器从不存在的状态规划，不能因此宣称可行。

代价中的加速度差惩罚只鼓励平滑。**jerk** 是加速度的变化率；这里没有硬约束 $|(a_k-a_{k-1})/T|\le j_{max}$，也没有力矩约束、碰撞约束、松弛变量或保证递归可行的终端集合。力矩在 MIT 执行层最后限幅；预测接触在求解后另行检查。即使当前 q/v 在界内，也可能因为刹车距离不足而无可行解。

### 3.5 怎样变成 OSQP 的矩阵

OSQP 接收

$$
\min_z\ \tfrac12z^TPz+c^Tz,\qquad l\le Gz\le u.
$$

$P$ 装平方代价，$c$ 装目标和上一输入带来的线性项，$G$ 装动力学等式及变量选择，$l,u$ 装对应上下界。与最优解无关的常数可以省略。

先看单关节、两步的约束排列。此时 $x_k$ 为二维，$z=[x_0;x_1;x_2;a_0;a_1]$ 共八个量。动力学等式块为

$$
E=\begin{bmatrix}
I_2&0&0&0&0\\
-A&I_2&0&-B&0\\
0&-A&I_2&0&-B
\end{bmatrix},\quad
Ez=\begin{bmatrix}x_{measured}\\0_2\\0_2\end{bmatrix}=b.
$$

再用选择矩阵 $F=[0_{6\times2}\ I_6]$ 从 $z$ 中抽出 $x_1,x_2,a_0,a_1$，给每个位置、速度、加速度分量加上下界。于是

$$
G=\begin{bmatrix}E\\F\end{bmatrix},\qquad
l=\begin{bmatrix}b\\\text{运动下界}\end{bmatrix},\quad
u=\begin{bmatrix}b\\\text{运动上界}\end{bmatrix}.
$$

等式通过令上下界相等实现。默认七轴 25 步有 364 行等式、525 行运动不等式，共 889 行。

对于输入序列 $a=[a_0;\ldots;a_{N-1}]$，令

$$
D=\begin{bmatrix}
I&0&\cdots&0\\
-I&I&\cdots&0\\
&\ddots&\ddots&\\
0&\cdots&-I&I
\end{bmatrix},\quad
b_a=[a_{prev};0;\ldots;0].
$$

则 $Da-b_a=[a_0-a_{prev};a_1-a_0;\ldots]$。定义 $\bar Q=\operatorname{diag}(Q,\ldots,Q,Q_f)$，$\bar R=I_N\otimes R$，$\bar S=I_N\otimes S$，$X_g=[x_g;\ldots;x_g]$，展开代价得到

$$
P=2\operatorname{diag}(\bar Q,\bar R+D^T\bar S D),\qquad
c=\begin{bmatrix}-2\bar QX_g\\-2D^T\bar S b_a\end{bmatrix}.
$$

系数 2 抵消 OSQP 目标前面的 $1/2$。因为 $b_a$ 只有首块非零，输入线性项只有第一块增加 $-2S a_{prev}$。这正是 `vectors()` 中的

```python
linear[self.ns:self.ns+n] -= 2*self.c['delta_a_weight']*np.asarray(previous_a)
l[:self.nx] = u[:self.nx] = np.r_[q, dq]
```

`LinearMPC.solve()` 调用 `solver.update(q=linear, l=l, u=u)`。再次注意：**OSQP 参数 `q` 表示这里的 $c$，不是机器人位置 $q$**。

### 3.6 热启动、收敛与接受条件

`LinearMPC.__init__()` 创建一次 OSQP 求解器，传入 CSC 格式的上三角 $P$ 和 $G$，设置 `warm_starting=True`、`polishing=True`。首次没有显式传入初始猜测，依赖 OSQP 默认初始化；后续使用求解器自动保留的原始/对偶结果热启动。当前代码没有把控制序列向前移位，也没有手工把旧轨迹改造成新初值。

每周期只更新 $c,l,u$，不更新 $P,G$。OSQP 用二次规划的结构反复协调代价最优性和约束满足程度，而不是枚举轨迹。`polishing` 在主迭代后尝试改善解的精度。

| 检查量 | 通俗含义 | 当前处理 |
|---|---|---|
| 求解状态 | 求解器认为成功、精度不足、不可行还是迭代耗尽 | 只考虑状态 1 `solved`、2 `solved inaccurate` |
| 原始残差 | 约束相关变量是否足够一致 | `prim_res <= 0.001` |
| 对偶残差 | 当前点离一阶最优性条件还有多远 | `dual_res <= 0.001` |
| 应用层约束违反 | 直接重算 $Gz$，最多越出 $[l,u]$ 多少 | `violation <= 0.001`，同时要求解全部有限 |
| 不等式 margin | 未来状态和输入距最近上下界的余量 | 记录供分析，不另设接受阈值 |

求解器 `eps_abs=eps_rel=1e-5` 参与自身带尺度的停止规则；应用层 `qp_acceptance=0.001` 是额外复核，不是同一个参数。这些标量跨不同单位的约束行使用，不能都解释为某一种关节物理误差。`max_iter=10000` 只限制迭代次数，不保证墙钟实时。初值无效、无解或复核失败时返回 `success=False`，不输出可执行预测；主循环转入参考制动并累计失败，详见 [架构](architecture.md)。

### 3.7 实际核算的单关节、两步例子

以下是**教学简化**：改成单关节，$N=2,T=1$ s，初始 $q_0=v_0=0$，目标 $q_g=1$ rad，上一加速度为 0。为了看清数字，时间步比项目默认大；其余代价权重与上表一致。位置界取 $\pm10$ rad、速度界取 $\pm10$ rad/s，不代表真实 Panda 参数。

由同一个预测模型直接得到

$$
q_1=0.5a_0,\quad v_1=a_0,\quad
q_2=1.5a_0+0.5a_1,\quad v_2=a_0+a_1.
$$

代入代价：

$$
\begin{aligned}
J={}&100+100(0.5a_0-1)^2+8a_0^2\\
&+0.15(a_0^2+a_1^2)+0.3[a_0^2+(a_1-a_0)^2]\\
&+400(1.5a_0+0.5a_1-1)^2+32(a_0+a_1)^2.
\end{aligned}
$$

用现有 `LinearMPC` 实际求解两种加速度界，并按上述公式重算代价，得到：

| 加速度界 | $a_0,a_1$ (rad/s²) | $q_1,q_2$ (rad) | $v_1,v_2$ (rad/s) | 完整代价 $J$ |
|---|---|---|---|---:|
| $\pm10$，本例未触及 | 1.104189，−1.255262 | 0.552094，1.028652 | 1.104189，−0.151073 | 133.329635 |
| $\pm0.5$ | 0.500000，0.257833 | 0.250000，0.878917 | 0.500000，0.757833 | 182.632498 |

较宽边界允许先加速再减速；收紧后第一步加速度被限制，优化器重新权衡终点位置与速度，第二步仍选择正加速度。它不会把原解两个输入分别裁剪一下就结束。终端速度非零也说明软代价不等于“窗口结束必须到达并停稳”。作为参照，两步都不动时完整代价为 600。

下面的短片段可重现输入和预测，调用的是项目代码，没有另写一套 MPC：

```python
import numpy as np
from panda_mpc.config import load_config
from panda_mpc.mpc import LinearMPC

c = load_config(horizon=2, period=1.0)
for bound in (10.0, 0.5):
    mpc = LinearMPC(np.array([-10.]), np.array([10.]),
                    np.array([10.]), np.array([bound]), c)
    r = mpc.solve(np.zeros(1), np.zeros(1), np.ones(1), np.zeros(1))
    assert r.success
    print(r.a_pred.ravel(), r.q_pred.ravel(), r.dq_pred.ravel())
```

执行时仅采用 $a_0$ 生成本段参考；下一次重新测量后求新的两步序列。$a_1$ 用于评估未来，不是承诺下一周期一定照搬。

## 4. 连续参考：为什么不直接连接预测点

MPC 每周期从实际测量开始，但 MIT 正在追踪上一段参考，两者通常有小偏差。如果新段也直接从测量起步，位置或速度命令会跳；如果保持旧参考起点，却硬在 20 ms 内追上新的预测终点，微小位置偏差可能按 $1/T^2$ 放大成很大的所需加速度。

[ReferenceGenerator.advance](../src/panda_mpc/reference.py) 的做法是：取上一**接受参考**在接缝的 $q_s,v_s$，将新 MPC 的第一组加速度 $a_0$ 从这里积分。以下公式针对未回退、模型等式理想满足的段，局部时间 $0\le s\le T$：

$$
q_{ref}(s)=q_s+v_ss+\tfrac12a_0s^2,\qquad
v_{ref}(s)=v_s+a_0s,\qquad a_{ref}(s)=a_0.
$$

在 $s=0$ 时，它严格接上旧参考的 q/v；这就是 **C1 连续**，即位置及其一阶导数速度连续。下一段可选择不同加速度，所以不保证加速度连续，也没有硬 jerk 限制。

### 4.1 预测点与下发参考差多少

MPC 的第一预测点是

$$
q_{pred}[1]=q_{measured}+Tv_{measured}+\tfrac12T^2a_0.
$$

将它从 $q_{ref}(T)$ 中减去，相同加速度项相消：

$$
q_{ref}(T)-q_{pred}[1]
=(q_s-q_{measured})+T(v_s-v_{measured}).
$$

速度差同样为 $v_{ref}(T)-v_{pred}[1]=v_s-v_{measured}$。实际数值还会有 QP 等式残差；若参考回退，接受的加速度也改变，上式还需计入该输入差异。因此不要把 `q_des` 与 `q_pred` 当作同一组数，也不能因为预测满足边界就跳过参考检查。

这项设计避免了历史版本强行追平预测终点引起的反复制动，但保留了“优化预测与实际下发参考不完全一致”的限制。下一次 MPC 仍使用实测 q/dq，绝不会把参考冒充测量。它也不因此自动证明参考偏差总能收敛。

### 4.2 为什么代码仍然使用三次 Hermite

**Hermite 曲线**是用起终点的位置和速度确定的多项式。`ReferenceSegment.hermite()` 统一表示为

$$
q_{ref}(s)=c_0+c_1s+c_2s^2+c_3s^3,
$$

$$
c_0=q_s,\quad c_1=v_s,\quad
c_2=\frac{3(q_e-q_s)}{T^2}-\frac{2v_s+v_e}{T},\quad
c_3=-\frac{2(q_e-q_s)}{T^3}+\frac{v_s+v_e}{T^2}.
$$

对同一曲线求导：

$$
v_{ref}=c_1+2c_2s+3c_3s^2,\qquad a_{ref}=2c_2+6c_3s.
$$

默认 `advance()` 令 $q_e=q_s+Tv_s+T^2a_0/2$、$v_e=v_s+Ta_0$，代入得到 $c_2=a_0/2,c_3=0$。所以恒加速度是三次表示的二次特例，不存在额外互相矛盾的位置、速度、加速度插值器。

```python
qs, vs, _ = self.boundary(t)
qe = qs+self.T*vs+0.5*self.T**2*acceleration
ve = vs+self.T*acceleration
return self.update(t, qe, ve, goal_id)
```

### 4.3 整段检查、回退与制动

只检查端点可能漏掉中间超界。`extrema()` 除端点外，还求 $v(s)=0$ 的根检查位置极值，求 $a(s)=0$ 的根检查速度极值；加速度为线性函数，端点足以覆盖其极值。`legal()` 用这些候选点复核整段 q/v/a 及有限性，容许 $10^{-7}$ 数值误差。

候选段不合法时，`update()` 先计算受界限制的制动加速度

$$
a_b=\operatorname{clip}(-v_s/T,-a_{max},a_{max}),
\quad q_b=q_s+Tv_s+\tfrac12T^2a_b,\quad v_b=v_s+Ta_b.
$$

随后固定起点，按 $\alpha=1,0.5,0.25,0.125,0$ 依次尝试混合终点

$$
q_e^{(\alpha)}=\alpha q_e+(1-\alpha)q_b,\qquad
v_e^{(\alpha)}=\alpha v_e+(1-\alpha)v_b.
$$

每个候选都重新构造完整曲线并检查；首个合法段才被接受。求解失败触发的显式制动直接把终点设为 $(q_b,v_b)$。这里裁剪的是制动加速度候选，不逐点裁剪参考，因而保留导数关系。制动可能需要多个周期，不保证一段内停下；若连纯制动也越界，抛出异常并停止，不伪造合法参考。

## 5. MIT 执行：位置和速度怎样变成力矩

参考只是“希望机器人怎样运动”。[mit.compose](../src/panda_mpc/mit.py) 把它与实测状态比较，通过关节阻抗形成力矩：位置误差像弹簧产生纠偏力，速度误差像阻尼抑制运动差异。

$$
\tau_{raw}=K_p(q_{des}-q)+K_d(v_{des}-v)+\tau_{ff},\qquad
\tau_{applied}=\operatorname{clip}(\tau_{raw},-\tau_{max},\tau_{max}).
$$

第一项是位置反馈，第二项是速度反馈，第三项用模型预先估算运动需要的力矩。这里的增益逐轴相乘，等效为对角矩阵。$K_p$ 用 Nm/rad，$K_d$ 用 Nm·s/rad，三项都输出 Nm；反馈没有再乘质量矩阵。

默认 `kp=[180,180,160,140,60,50,30]`，`kd=[30,30,25,22,10,8,6]`；力矩限幅为前四轴 70、后三轴 10 Nm。这些参数在已有实验中使用，不构成任意负载下的稳定性保证。

### 5.1 三种名义前馈

[NominalDynamics.feedforward](../src/panda_mpc/dynamics.py) 输入**实测 q/dq**和**参考加速度**，在独立模型及计算数据上生成前馈：

| 模式 | 返回量 | 意义 |
|---|---|---|
| `none` | 零 | 全靠位置/速度反馈 |
| `gravity` | 名义数据置零速度后读取 `qfrc_bias` | 估计重力补偿；不会把真实速度清零 |
| `inverse_dynamics`，默认 | $M(q)a_{ref}+qfrc\_bias-qfrc\_passive$ | 估计惯性、重力/科氏等偏置及被动力的作用 |

MuJoCo 自由空间动力学的符号关系可写为

$$
M(q)\ddot q+qfrc\_bias=\tau+qfrc\_passive.
$$

所以要得到参考加速度，所需主动输入为

$$
\tau_{ff}=M(q)a_{ref}+qfrc\_bias-qfrc\_passive.
$$

被动力在动力学右边已经提供力矩，因此从所需主动输入中**减去**它；例如被动阻尼自身阻碍运动，前馈会补偿其作用。这里是名义自由空间计算，未把真实接触约束力、外力或执行器力读入并抵消。

```python
mujoco.mj_mulM(self.model, self.data, inertial, np.asarray(ddq))
return inertial + self.data.qfrc_bias - self.data.qfrc_passive
```

`mj_mulM` 做质量矩阵与参考加速度的乘法，包含模型的关节 armature。前馈不再叠加位置/速度反馈；反馈只在 MIT 层算一次，否则会无意中改变总刚度和阻尼。

### 5.2 总力矩、有效期与电机语义

`MITAdapter.apply()` 先调用 `compose()`，对反馈加前馈的**总力矩**限幅，随后唯一一次写入 `data.ctrl`：

```python
raw = p + v + ff
applied = np.clip(raw, -limits, limits)
# MITAdapter.apply 中：
data.ctrl[self.robot.actuators] = result.tau_applied
```

为什么这里 `ctrl` 可以表示 Nm？随包模型用七个 motor，gear=1、固定 gain=1、无 bias 和激活动态，加载时核对了这些条件；其模型限幅不小于配置的运行限幅。没有隐藏的第二套位置伺服再做同样的 PD。

`JointCommand` 除五个七维向量外还包含时间戳、有效期及目标编号。主循环令命令有效到当前参考段终点；过期或来自未来的命令转为 $-K_dv$ 阻尼，位置反馈和前馈清零，再做总力矩限幅。该故障处理不保证与旧命令连续。

这里的 MIT 是仿真适配器的关节阻抗接口语义，不是 Panda 原生五参数硬件通信，也未实现 CAN、设备驱动或真实硬件安全控制。

## 6. 动态响应能力的边界

新目标使 IK 更新终点，MPC 从当时实测状态重新规划；外力则先改变物理运动，再通过每 2 ms 的 MIT 反馈以及下一次 MPC 测量进入控制。因此扰动恢复由执行反馈、重规划和名义前馈共同影响，不能全部归因于 MPC。

当前没有在线参数辨识或参数自适应律，也没有证明任意模型偏差下稳定。接触检查会拒绝终点或离散预测构型，不能替代全局绕障规划或连续路径无碰撞证明。任务主要是离散目标点到达与替换，尚未实现给定整条连续目标轨迹的预瞄跟踪。理解这些边界后再结合 [实验指标](validation.md) 判断效果，避免只看一条末端曲线得出过大的结论。
