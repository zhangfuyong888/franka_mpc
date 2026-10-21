"""Offline numerical evidence using the locally installed CasADi OSQP C plugin.

This exercises the production sparse QP assembly, NOT the production Python OSQP
wrapper and NOT a MuJoCo integration test. It is intentionally outside the package.
"""
import argparse
import json
from pathlib import Path
from time import perf_counter
import casadi as ca
import numpy as np
from panda_mpc.config import load_config
from panda_mpc.mpc import MPCProblem


def experiment(n=1, horizon=25, smoothness=0.3):
    c = load_config(horizon=horizon, delta_a_weight=smoothness)
    p = MPCProblem(-np.ones(n), np.ones(n), np.full(n, 0.7), np.full(n, 2.), c)
    def dm(matrix):
        coo = matrix.tocoo()
        pattern = ca.Sparsity.triplet(*coo.shape, coo.row.tolist(), coo.col.tolist())
        # Assign through dense here to avoid SciPy/CasADi storage-order assumptions.
        return ca.project(ca.DM(matrix.toarray()), pattern)
    H, A = dm(p.P), dm(p.G)
    solver = ca.conic('offline_mpc', 'osqp', {'h': H.sparsity(), 'a': A.sparsity()},
                       {'error_on_fail': True, 'osqp': {'verbose': False, 'eps_abs': 1e-7,
                           'eps_rel': 1e-7, 'max_iter': 20000, 'polish': True}})
    q, v, prev = np.zeros(n), np.zeros(n), np.zeros(n)
    rows, max_violation, solve_times = [], 0., []
    max_delta_a, peak_v, peak_a = 0., 0., 0.
    for k in range(400):
        goal = np.full(n, 0.6 if k < 100 else -0.4)
        linear, l, u = p.vectors(q, v, goal, prev)
        start = perf_counter()
        r = solver(h=H, g=linear, a=A, lba=l, uba=u)
        solve_times.append(perf_counter()-start)
        z = np.asarray(r['x']).ravel()
        violation, _ = p.inspect(z, l, u)
        max_violation = max(max_violation, violation)
        assert violation < 1e-5, violation
        assert np.max(np.abs(z[:p.nx]-np.r_[q, v])) < 1e-5
        a = z[p.ns:p.ns+n]
        max_delta_a = max(max_delta_a, float(np.max(np.abs(a-prev))))
        peak_a = max(peak_a, float(np.max(np.abs(a))))
        rows.append(np.r_[k*p.T, q, v, a, goal])
        q = q+p.T*v+0.5*p.T*p.T*a
        v = v+p.T*a
        peak_v = max(peak_v, float(np.max(np.abs(v))))
        prev = a
    assert np.max(np.abs(q+0.4)) < 0.01
    assert np.max(np.abs(v)) < 0.02
    return {'n': n, 'horizon': horizon, 'S': smoothness, 'nz': p.nz,
            'final_position_error': float(np.max(np.abs(q+0.4))),
            'final_velocity': float(np.max(np.abs(v))), 'peak_velocity': peak_v,
            'peak_acceleration': peak_a, 'max_input_change': max_delta_a,
            'max_constraint_violation': max_violation,
            'solve_p50_ms': float(np.percentile(solve_times, 50)*1000),
            'solve_p95_ms': float(np.percentile(solve_times, 95)*1000),
            'solve_max_ms': float(np.max(solve_times)*1000)}, np.asarray(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='docs/evidence/native_osqp')
    args = parser.parse_args()
    out = Path(args.output); out.mkdir(parents=True, exist_ok=True)
    results = []
    for n, N, S in [(1, 25, 0.3), (7, 25, 0.3), (1, 15, 0.3), (1, 25, 3.)]:
        metrics, rows = experiment(n, N, S)
        np.savez_compressed(out/f'n{n}-N{N}-S{S}.npz', trajectory=rows)
        print(json.dumps(metrics)); results.append(metrics)
    (out/'summary.json').write_text(json.dumps({'backend': 'CasADi 3.6.7 OSQP C plugin; not Python OSQP or MuJoCo', 'experiments': results}, indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 7))
    for N, S in [(25, 0.3), (15, 0.3), (25, 3.)]:
        data = np.load(out/f'n1-N{N}-S{S}.npz')['trajectory']
        for ax, col in zip(axes, [1, 2, 3]):
            ax.plot(data[:, 0], data[:, col], label=f'N={N}, S={S}')
    axes[0].plot(data[:, 0], data[:, 4], '--', label='goal')
    for ax, unit in zip(axes, ['q [rad]', 'v [rad/s]', 'a [rad/s²]']):
        ax.set_ylabel(unit); ax.legend()
    axes[-1].set_xlabel('ideal model time [s]')
    fig.tight_layout(); fig.savefig(out/'comparison.png', dpi=130); plt.close(fig)


if __name__ == '__main__':
    main()
