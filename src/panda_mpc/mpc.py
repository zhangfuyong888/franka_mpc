"""Sparse acceleration-level double-integrator MPC; no dynamics or MIT coupling."""
from time import perf_counter
import numpy as np
from scipy import sparse as sp
from .types import MPCResult


class MPCProblem:
    """Assembly is solver-independent, supporting explicit numerical verification."""
    def __init__(self, lower, upper, velocity, acceleration, config):
        self.lower, self.upper = np.asarray(lower), np.asarray(upper)
        self.velocity, self.acceleration = np.asarray(velocity), np.asarray(acceleration)
        self.n = len(self.lower)
        self.nx = 2*self.n
        self.N, self.T = config['horizon'], config['period']
        self.ns = (self.N+1)*self.nx
        self.nz = self.ns+self.N*self.n
        self.c = config
        n, nx, N, T = self.n, self.nx, self.N, self.T
        eye = sp.eye(n, format='csc')
        self.Ad = sp.bmat([[eye, T*eye], [None, eye]], format='csc')
        self.Bd = sp.vstack([0.5*T*T*eye, T*eye], format='csc')
        qdiag = np.r_[np.full(n, config['q_weight']), np.full(n, config['v_weight'])]
        self.state_weights = np.r_[np.tile(qdiag, N), config['terminal_scale']*qdiag]
        Q = sp.diags(self.state_weights, format='csc')
        D = sp.kron(sp.eye(N)-sp.diags(np.ones(N-1), -1, shape=(N, N)), eye, format='csc')
        H = config['a_weight']*sp.eye(N*n) + config['delta_a_weight']*(D.T@D)
        self.P = 2*sp.block_diag([Q, H], format='csc')
        # x0 = measured x, then x[k+1] - Ad*x[k] - Bd*a[k] = 0.
        E = sp.lil_matrix(((N+1)*nx, self.nz))
        E[:nx, :nx] = sp.eye(nx)
        for k in range(N):
            row = slice((k+1)*nx, (k+2)*nx)
            E[row, k*nx:(k+1)*nx] = -self.Ad
            E[row, (k+1)*nx:(k+2)*nx] = sp.eye(nx)
            E[row, self.ns+k*n:self.ns+(k+1)*n] = -self.Bd
        self.neq = (N+1)*nx
        # Identity selectors for x[1..N], a[0..N-1]; x0 is never clipped.
        F = sp.eye(self.nz, format='csc')[nx:]
        self.G = sp.vstack([E.tocsc(), F], format='csc')
        lo = np.r_[np.tile(np.r_[self.lower, -self.velocity], N), np.tile(-self.acceleration, N)]
        hi = np.r_[np.tile(np.r_[self.upper, self.velocity], N), np.tile(self.acceleration, N)]
        self.l = np.r_[np.zeros(self.neq), lo]
        self.u = np.r_[np.zeros(self.neq), hi]

    def vectors(self, q, dq, goal, previous_a):
        n = self.n
        goal = np.asarray(goal)
        linear = np.r_[-2*self.state_weights*np.tile(np.r_[goal, np.zeros(n)], self.N+1),
                       np.zeros(self.N*n)]
        linear[self.ns:self.ns+n] -= 2*self.c['delta_a_weight']*np.asarray(previous_a)
        l, u = self.l.copy(), self.u.copy()
        l[:self.nx] = u[:self.nx] = np.r_[q, dq]
        return linear, l, u

    def inspect(self, z, l, u):
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
        if (np.any(q < p.lower) or np.any(q > p.upper)
                or np.any(np.abs(dq) > p.velocity)):
            return MPCResult(False, 'measured_state_outside_operating_bounds', elapsed=perf_counter()-start)
        linear, l, u = p.vectors(q, dq, goal, previous_a)
        self.solver.update(q=linear, l=l, u=u)
        result = self.solver.solve(raise_error=False)
        info = result.info
        elapsed = perf_counter()-start
        violation, margin = float('inf'), float('-inf')
        finite = result.x is not None and np.isfinite(result.x).all()
        if finite:
            violation, margin = p.inspect(result.x, l, u)
        tol = p.c['qp_acceptance']
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
