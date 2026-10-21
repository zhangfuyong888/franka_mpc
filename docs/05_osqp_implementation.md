# 05：从代价到实际稀疏 QP

MPCProblem 构造 $\frac12z^TPz+c^Tz$、$l\le Gz\le u$。$z=[x_0,\ldots,x_N,a_0,\ldots,a_{N-1}]$，`ns=(N+1)*nx`、`nz=ns+N*n`。七轴 N25 为 539，单轴为 77。

令 $\bar Q=\operatorname{diag}(Q,\ldots,Q,Q_f)$，阶段状态 N 块、终端一块。$\bar R=I_N\otimes R$，块差分矩阵

$$D=\begin{bmatrix}I&0&\cdots\\-I&I&\cdots\\0&-I&\ddots\end{bmatrix},\quad b=[a_{previous};0;\ldots;0].$$

展开 $(Da-b)^T\bar S(Da-b)$，得到二次项 $a^TD^T\bar SDa$ 和线性项 $-2b^T\bar SDa$，忽略常数。故

$$P=2\operatorname{diag}(\bar Q,\bar R+D^T\bar SD),$$
$$c_x=-2\bar Q[x_g;\ldots;x_g],\quad c_a=-2D^T\bar Sb.$$

当前 S 为标量乘 I，只有 a0 线性块出现 -2S a_previous。D^TSD 产生相邻输入交叉项。P 的 2 来自求解器的 0.5，不能遗漏。

E 第一 nx 行是 x0=测量，之后各动力学行为 [-Ad,I,-Bd]，界为零。F 是 `I_nz[nx:,:]`，选择 x1..xN 和全部 a，跳过 x0。界按 [q_lower,-vmax]/[q_upper,vmax] 重复 N 次，再接 ±amax。G=vstack(E,F)，均使用 CSC。

LinearMPC 一次 setup、之后每 tick update(q,l,u)。P/G 固定，OSQP 自动 warm start，没有复杂移位。使用上三角 P。代码按 OSQP1.0.4 的 warm_starting/polishing/solve(raise_error=False) 和 info.prim_res/dual_res 编写，但该 Python 版本没有安装，API 运行兼容性尚未验收。

接受条件：status_val 为 1 或 2、数组有限、最大约束违反≤0.001、原/对偶残差均≤0.001。solved inaccurate 不自动接受。elapsed 包含 Python update/solve 包装，另记录迭代与余量，不冒充整 tick 耗时。

已运行原代价与矩阵代价等价性、初值/动力学/目标线性项/上一输入项单元检查，另用真实 OSQP0.6.3 C 库求解相同矩阵。七轴 N25 的 400 次 CasADi 包装调用 P50≈1.01 ms、P95≈12.29 ms、最大≈23.11 ms；不代表生产 Python 后端或机器人整 tick 实时性。

| 数学 | 代码 |
|---|---|
| Ad/Bd/N/T | MPCProblem.Ad/Bd/N/T |
| Qbar | state_weights |
| D | 构造函数局部 D |
| P/G | MPCProblem.P/G |
| c/l/u 更新 | vectors |
| 违反与余量 | inspect |
| 求解器 | LinearMPC.solver |

例子 `python -m pytest -q tests/test_core.py -k qp` 已通过。常见错误：遗漏系数 2、约束错放 x0、遗漏 a0-a_previous 线性项。练习：手算 N2,n1 的 D^TD 与代码比较。
