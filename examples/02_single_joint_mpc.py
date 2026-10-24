"""单关节理想 MPC：2 s 换目标，自动保存轨迹、三联图和结果摘要。"""
import argparse
from pathlib import Path
from datetime import datetime
import numpy as np
from panda_mpc.config import load_config
from panda_mpc.mpc import LinearMPC
from panda_mpc.telemetry import write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--horizon', type=int, default=25)
    parser.add_argument('--smoothness', type=float, default=.3)
    parser.add_argument('--output', help='NPZ filename; default creates a timestamped experiment')
    args = parser.parse_args()
    output = Path(args.output or f"outputs/examples/{datetime.now():%Y%m%d-%H%M%S-%f}-single_joint_mpc.npz")
    if output.exists() or output.with_suffix('.png').exists() or output.with_suffix('.json').exists():
        parser.error('Output already exists; choose a new --output filename')
    output.parent.mkdir(parents=True, exist_ok=True)
    config = load_config(horizon=args.horizon, delta_a_weight=args.smoothness)
    controller = LinearMPC(np.array([-1.]), np.array([1.]), np.array([.7]), np.array([2.]), config)
    q, velocity, previous_a = np.zeros(1), np.zeros(1), np.zeros(1)
    rows = []
    for tick in range(400):
        goal = np.array([.6 if tick < 100 else -.4])
        result = controller.solve(q, velocity, goal, previous_a)
        if not result.success:
            write_json(output.with_suffix('.failure.json'), {
                'tick': tick, 't': tick*config['period'], 'status': result.status,
                'q': q, 'velocity': velocity, 'failure_kind': result.failure_kind,
                'note': 'Measured/ideal state is preserved; no clipping to fake feasibility',
            })
            raise RuntimeError(f'{result.status}; see {output.with_suffix(".failure.json")}')
        rows.append(np.r_[tick*config['period'], q, velocity, result.a_pred[0], goal])
        # 本例是理想双积分仿真；这里采用预测下一点，不代表真实机械臂会完全照此运动。
        q, velocity, previous_a = result.q_pred[1], result.dq_pred[1], result.a_pred[0]
    trajectory = np.array(rows)
    np.savez_compressed(output, trajectory=trajectory)
    from panda_mpc.analyze import plt
    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(trajectory[:, 0], trajectory[:, 1], label='joint position')
    axes[0].step(trajectory[:, 0], trajectory[:, 4], '--', label='goal')
    axes[0].set_ylabel('position [rad]'); axes[0].legend()
    for ax, column, label, limit in [(axes[1], 2, 'velocity [rad/s]', .7),
                                      (axes[2], 3, 'acceleration [rad/s2]', 2.)]:
        ax.plot(trajectory[:, 0], trajectory[:, column])
        ax.axhline(limit, color='gray', ls='--'); ax.axhline(-limit, color='gray', ls='--')
        ax.set_ylabel(label)
    for ax in axes:
        ax.axvline(2., color='gray', ls=':')
    axes[-1].set_xlabel('time [s]'); fig.tight_layout()
    fig.savefig(output.with_suffix('.png'), dpi=130); plt.close(fig)
    summary = {'model': 'ideal single-joint double integrator, not MuJoCo',
               'horizon': args.horizon, 'smoothness': args.smoothness,
               'final_error_rad': float(abs(q[0]-goal[0])), 'final_speed_rad_s': float(abs(velocity[0])),
               'peak_speed_rad_s': float(abs(trajectory[:, 2]).max()),
               'peak_acceleration_rad_s2': float(abs(trajectory[:, 3]).max())}
    write_json(output.with_suffix('.json'), summary)
    print(summary)
    print(f'Data: {output}; plot: {output.with_suffix(".png")}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
