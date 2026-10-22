"""关节加速度级线性 MPC：以实测状态预测运动，不在 QP 中求解力矩。

变量与矩阵推导见 docs/control_math.md 的 MPC 部分。
"""
from time import perf_counter
import numpy as np
from scipy import sparse as sp
from .types import MPCResult


class MPCProblem:
    """组装稀疏 QP；与求解器分离，方便测试直接核对代价和动力学。"""
    def __init__(self, lower, upper, velocity, acceleration, config):
        self.lower, self.upper = np.asarray(lower), np.asarray(upper)
        self.velocity, self.acceleration = np.asarray(velocity), np.asarray(acceleration)
        self.n = len(self.lower)
        self.nx = 2*self.n
        self.N, self.T = config['horizon'], config['period']
        # z = [x0, …, xN, a0, …, a(N-1)]，x=[q; v]。
        # 七轴、25 步时：状态 26×14=364 个，输入 25×7=175 个，共 539 个变量。
        self.ns = (self.N+1)*self.nx
        self.nz = self.ns+self.N*self.n
        self.c = config
        n, nx, N, T = self.n, self.nx, self.N, self.T
        eye = sp.eye(n, format='csc')
        # 恒加速度离散模型：q_next=q+T*v+T²*a/2，v_next=v+T*a。
        # 默认七轴时 Ad 为 14×14，Bd 为 14×7；q/v/a 单位为 rad、rad/s、rad/s²。
        self.Ad = sp.bmat([[eye, T*eye], [None, eye]], format='csc')
        self.Bd = sp.vstack([0.5*T*T*eye, T*eye], format='csc')
        qdiag = np.r_[np.full(n, config['q_weight']), np.full(n, config['v_weight'])]
        self.state_weights = np.r_[np.tile(qdiag, N), config['terminal_scale']*qdiag]
        Q = sp.diags(self.state_weights, format='csc')
        # D*a = [a0, a1-a0, …]；首项减 previous_a 的部分放到线性项中。
        D = sp.kron(sp.eye(N)-sp.diags(np.ones(N-1), -1, shape=(N, N)), eye, format='csc')
        H = config['a_weight']*sp.eye(N*n) + config['delta_a_weight']*(D.T@D)
        # OSQP 使用 0.5*z.T*P*z，故平方代价的 Hessian 需要乘 2。
        self.P = 2*sp.block_diag([Q, H], format='csc')
        # E 的首块固定 x0=实测状态，后续每块约束 x[k+1]-Ad*x[k]-Bd*a[k]=0。
        E = sp.lil_matrix(((N+1)*nx, self.nz))
        E[:nx, :nx] = sp.eye(nx)
        for k in range(N):
            row = slice((k+1)*nx, (k+2)*nx)
            E[row, k*nx:(k+1)*nx] = -self.Ad
            E[row, (k+1)*nx:(k+2)*nx] = sp.eye(nx)
            E[row, self.ns+k*n:self.ns+(k+1)*n] = -self.Bd
        self.neq = (N+1)*nx
        # F 从 z 中选择未来状态 x1…xN 和所有输入，加入位置/速度/加速度界。
        # x0 已由等式固定；不能裁剪测量值来制造可行解。
        F = sp.eye(self.nz, format='csc')[nx:]
        self.G = sp.vstack([E.tocsc(), F], format='csc')
        lo = np.r_[np.tile(np.r_[self.lower, -self.velocity], N), np.tile(-self.acceleration, N)]
        hi = np.r_[np.tile(np.r_[self.upper, self.velocity], N), np.tile(self.acceleration, N)]
        self.l = np.r_[np.zeros(self.neq), lo]
        self.u = np.r_[np.zeros(self.neq), hi]

    def vectors(self, q, dq, goal, previous_a):
        """更新目标线性项及初值等式；previous_a 是上一接受参考的边界加速度。"""
        n = self.n
        goal = np.asarray(goal)
        linear = np.r_[-2*self.state_weights*np.tile(np.r_[goal, np.zeros(n)], self.N+1),
                       np.zeros(self.N*n)]
        # S*||a0-previous_a||² 展开后，只有 a0 的线性项增加 -2*S*previous_a。
        linear[self.ns:self.ns+n] -= 2*self.c['delta_a_weight']*np.asarray(previous_a)
        l, u = self.l.copy(), self.u.copy()
        l[:self.nx] = u[:self.nx] = np.r_[q, dq]
        return linear, l, u

    def inspect(self, z, l, u):
        """复核 Gz：violation 包含等式与不等式，margin 仅统计运动边界余量。"""
        g = self.G @ z
        violation = max(0., float(np.max(l-g)), float(np.max(g-u)))
        ineq = g[self.neq:]
        margin = float(np.min(np.r_[ineq-l[self.neq:], u[self.neq:]-ineq]))
        return violation, margin


class LinearMPC:
    def __init__(self, lower, upper, velocity, acceleration, config):
        import osqp
        self.problem = MPCProblem(lower, upper, velocity, acceleration, config)
        p = self.problem
        self.solver = osqp.OSQP()
        # 首次初值依赖 OSQP 默认初始化；后续自动复用解，没有手动移位控制序列。
        self.solver.setup(P=sp.triu(p.P, format='csc'), q=np.zeros(p.nz), A=p.G,
                          l=p.l, u=p.u, verbose=False, warm_starting=True,
                          eps_abs=config['qp_tolerance'], eps_rel=config['qp_tolerance'],
                          max_iter=10000, polishing=True)

    def solve(self, q, dq, goal, previous_a):
        p = self.problem
        start = perf_counter()
        values = [np.asarray(v, float) for v in (q, dq, goal, previous_a)]
        if any(v.shape != (p.n,) or not np.isfinite(v).all() for v in values):
            return MPCResult(False, 'invalid_input', elapsed=perf_counter()-start)
        q, dq, goal, previous_a = values
        # 只容忍活跃边界上的浮点舍入误差，仍将原始实测值写入 x0 等式。
        bound_roundoff = 1e-10
        if (np.any(q < p.lower-bound_roundoff) or np.any(q > p.upper+bound_roundoff)
                or np.any(np.abs(dq) > p.velocity+bound_roundoff)):
            return MPCResult(False, 'measured_state_outside_operating_bounds', elapsed=perf_counter()-start)
        linear, l, u = p.vectors(q, dq, goal, previous_a)
        # P、G 的结构和数值固定；每周期只更新 c、l、u。接口 q 指线性系数 c，
        # 与本函数的机器人关节位置 q 不同。
        self.solver.update(q=linear, l=l, u=u)
        result = self.solver.solve(raise_error=False)
        info = result.info
        elapsed = perf_counter()-start
        violation, margin = float('inf'), float('-inf')
        finite = result.x is not None and np.isfinite(result.x).all()
        if finite:
            violation, margin = p.inspect(result.x, l, u)
        tol = p.c['qp_acceptance']
        # solved / solved inaccurate 都需通过有限性、残差和 Gz 越界复核；
        # 应用接受阈值不等同于 OSQP 自身 eps_abs/eps_rel 停止条件。
        accepted = (info.status_val in (1, 2) and finite and violation <= tol
                    and info.prim_res <= tol and info.dual_res <= tol)
        r = MPCResult(accepted, info.status, primal_residual=float(info.prim_res),
                      dual_residual=float(info.dual_res), iterations=int(info.iter),
                      elapsed=elapsed, violation=violation, margin=margin)
        if accepted:
            states = result.x[:p.ns].reshape(p.N+1, p.nx)
            r.q_pred, r.dq_pred = states[:, :p.n].copy(), states[:, p.n:].copy()
            r.a_pred = result.x[p.ns:].reshape(p.N, p.n).copy()
        return r
