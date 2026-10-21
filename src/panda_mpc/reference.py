"""Cubic Hermite segments with exact interval extrema, bounded backtracking."""
from dataclasses import dataclass
import numpy as np


class ReferenceFailure(RuntimeError):
    pass


@dataclass
class ReferenceSegment:
    start: float
    duration: float
    coefficients: np.ndarray  # rows: constant, linear, quadratic, cubic; physical seconds
    goal_id: int
    alpha: float = 1.0
    reason: str = 'accepted'

    @classmethod
    def hermite(cls, start, T, qs, vs, qe, ve, goal_id, alpha=1.0, reason='accepted'):
        qs, vs, qe, ve = map(lambda x: np.asarray(x, float), (qs, vs, qe, ve))
        c2 = 3*(qe-qs)/T**2 - (2*vs+ve)/T
        c3 = -2*(qe-qs)/T**3 + (vs+ve)/T**2
        return cls(start, T, np.array([qs, vs, c2, c3]), goal_id, alpha, reason)

    def sample(self, t):
        s = min(max(t-self.start, 0.0), self.duration)
        c0, c1, c2, c3 = self.coefficients
        return c0+s*(c1+s*(c2+s*c3)), c1+s*(2*c2+3*s*c3), 2*c2+6*s*c3

    def extrema(self):
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
        return self.segment.sample(t)

    def update(self, t, qe, ve, goal_id, braking=False):
        qs, vs, _ = self.boundary(t)
        # Acceleration selected as a whole curve; samples are never clipped.
        brake_a = np.clip(-vs/self.T, -self.amax, self.amax)
        qb = qs+self.T*vs+0.5*self.T**2*brake_a
        vb = vs+self.T*brake_a
        if braking:
            qe, ve = qb, vb
        for alpha in (1., 0.5, 0.25, 0.125, 0.):
            segment = ReferenceSegment.hermite(t, self.T, qs, vs,
                        alpha*np.asarray(qe)+(1-alpha)*qb,
                        alpha*np.asarray(ve)+(1-alpha)*vb, goal_id, alpha,
                        'braking' if braking else ('accepted' if alpha == 1 else 'bounded_backtrack'))
            if segment.legal(self.lower, self.upper, self.vmax, self.amax):
                self.segment = segment
                return segment
        raise ReferenceFailure('No legal C1 segment, including bounded braking candidate')
