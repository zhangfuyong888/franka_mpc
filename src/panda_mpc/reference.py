"""从上一接受参考边界接续 C1 曲线，解析检查整段运动边界。

实测预测与下发参考的区别、Hermite 系数见 docs/control_math.md。
"""
from dataclasses import dataclass
import numpy as np


class ReferenceFailure(RuntimeError):
    pass


@dataclass
class ReferenceSegment:
    start: float
    duration: float
    coefficients: np.ndarray  # (4, n)：常数/一次/二次/三次项；时间变量用秒
    goal_id: int
    alpha: float = 1.0
    reason: str = 'accepted'

    @classmethod
    def hermite(cls, start, T, qs, vs, qe, ve, goal_id, alpha=1.0, reason='accepted'):
        qs, vs, qe, ve = map(lambda x: np.asarray(x, float), (qs, vs, qe, ve))
        # 四个边界条件 q(0)=qs、v(0)=vs、q(T)=qe、v(T)=ve 唯一确定三次系数。
        c2 = 3*(qe-qs)/T**2 - (2*vs+ve)/T
        c3 = -2*(qe-qs)/T**3 + (vs+ve)/T**2
        return cls(start, T, np.array([qs, vs, c2, c3]), goal_id, alpha, reason)

    def sample(self, t):
        """在同一条曲线上求位置、解析速度和解析加速度，避免三个参考各自拼接。"""
        s = min(max(t-self.start, 0.0), self.duration)
        c0, c1, c2, c3 = self.coefficients
        return c0+s*(c1+s*(c2+s*c3)), c1+s*(2*c2+3*s*c3), 2*c2+6*s*c3

    def extrema(self):
        """端点加导数零点覆盖整段极值：v=0 查位置，a=0 查速度，加速度是线性的。"""
        n = self.coefficients.shape[1]
        mins, maxs = np.empty((3, n)), np.empty((3, n))
        for j in range(n):
            _, c1, c2, c3 = self.coefficients[:, j]
            candidates = [0., self.duration]
            for poly in ([3*c3, 2*c2, c1], [6*c3, 2*c2]):
                a = np.trim_zeros(np.asarray(poly), 'f')
                if len(a) > 1:
                    for root in np.roots(a):
                        if abs(root.imag) < 1e-10 and 0 < root.real < self.duration:
                            candidates.append(float(root.real))
            values = np.array([np.array(self.sample(self.start+s))[:, j] for s in candidates])
            mins[:, j], maxs[:, j] = values.min(axis=0), values.max(axis=0)
        return mins, maxs

    def legal(self, lower, upper, vmax, amax, tolerance=1e-7):
        if not np.isfinite(self.coefficients).all():
            return False
        lo, hi = self.extrema()
        return bool(np.all(lo[0] >= lower-tolerance) and np.all(hi[0] <= upper+tolerance)
                    and np.all(lo[1] >= -np.asarray(vmax)-tolerance)
                    and np.all(hi[1] <= np.asarray(vmax)+tolerance)
                    and np.all(lo[2] >= -np.asarray(amax)-tolerance)
                    and np.all(hi[2] <= np.asarray(amax)+tolerance))


class ReferenceGenerator:
    def __init__(self, state, lower, upper, vmax, amax, T):
        self.lower, self.upper, self.vmax, self.amax = map(np.asarray, (lower, upper, vmax, amax))
        self.T = T
        self.segment = ReferenceSegment.hermite(state.t, T, state.q, state.dq,
                                                state.q+T*state.dq, state.dq, -1)

    def boundary(self, t):
        """上一接受曲线的 q/v/a；这是参考状态，不是物理测量。"""
        return self.segment.sample(t)

    def advance(self, t, acceleration, goal_id):
        """输入 MPC 第一组加速度 (n, rad/s²)，从上一参考边界积分出下一段。

        MPC 仍从实测 q/dq 预测；这里从参考 qs/vs 起步，因此端点一般不等于
        q_pred[1]/dq_pred[1]。强行在 20 ms 内追平两者偏差会放大所需加速度。
        恒加速度段是三次 Hermite 的二次特例；接缝 q/v 连续，a 不保证连续。
        """
        qs, vs, _ = self.boundary(t)
        acceleration = np.asarray(acceleration, dtype=float)
        qe = qs+self.T*vs+0.5*self.T**2*acceleration
        ve = vs+self.T*acceleration
        return self.update(t, qe, ve, goal_id)

    def update(self, t, qe, ve, goal_id, braking=False):
        qs, vs, _ = self.boundary(t)
        # 先构造受加速度界限制的制动曲线，再混合候选终点并逐段检查。
        # clip 只用于选制动加速度，不逐点裁剪 q/v/a，以免破坏导数关系。
        brake_a = np.clip(-vs/self.T, -self.amax, self.amax)
        qb = qs+self.T*vs+0.5*self.T**2*brake_a
        vb = vs+self.T*brake_a
        if braking:
            qe, ve = qb, vb
        # 固定起点 qs/vs，逐步向制动终点靠拢；连纯制动都不合法则由 runner 停止。
        for alpha in (1., 0.5, 0.25, 0.125, 0.):
            segment = ReferenceSegment.hermite(t, self.T, qs, vs,
                        alpha*np.asarray(qe)+(1-alpha)*qb,
                        alpha*np.asarray(ve)+(1-alpha)*vb, goal_id, alpha,
                        'braking' if braking else ('accepted' if alpha == 1 else 'bounded_backtrack'))
            if segment.legal(self.lower, self.upper, self.vmax, self.amax):
                self.segment = segment
                return segment
        raise ReferenceFailure('No legal C1 segment, including bounded braking candidate')
