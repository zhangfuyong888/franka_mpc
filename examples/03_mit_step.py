"""用解析小幅参考单独验证 MIT 跟踪，自动保存数据、跟踪图和力矩图。"""
import argparse
from pathlib import Path
from datetime import datetime
import numpy as np
from panda_mpc.config import load_config
from panda_mpc.sim import Simulation
from panda_mpc.dynamics import NominalDynamics
from panda_mpc.mit import MITAdapter
from panda_mpc.types import JointCommand
from panda_mpc.telemetry import write_json


def main():
    import xml.parsers.expat  # 先加载标准库，避免混合环境的图形库符号冲突。
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--feedforward', choices=['none', 'gravity', 'inverse_dynamics'], default='inverse_dynamics')
    parser.add_argument('--gain-scale', type=float, default=1.)
    parser.add_argument('--output', help='NPZ filename; creates matching PNG and JSON')
    args = parser.parse_args()
    if not 0 < args.gain_scale <= 2:
        parser.error('--gain-scale must be in (0, 2]')
    output = Path(args.output or f'outputs/examples/{datetime.now():%Y%m%d-%H%M%S-%f}-mit-{args.feedforward}-gain{args.gain_scale:g}.npz')
    if any(output.with_suffix(suffix).exists() for suffix in ('.npz', '.png', '.json')):
        parser.error('Output already exists; choose a new --output filename')
    output.parent.mkdir(parents=True, exist_ok=True)
    config = load_config(feedforward=args.feedforward)
    sim = Simulation(config)
    nominal = NominalDynamics(config['dt'])
    adapter = MITAdapter(sim.model, sim.robot, config['torque'])
    rows = []
    for tick in range(2500):
        state = sim.begin_step()
        duration, amplitude = 2., .1
        q_ref, v_ref, a_ref = sim.robot.home.copy(), np.zeros(7), np.zeros(7)
        q_ref[0] += amplitude*(1-np.cos(np.pi*min(state.t, duration)/duration))/2
        if state.t < duration:
            v_ref[0] = amplitude*np.pi/(2*duration)*np.sin(np.pi*state.t/duration)
            a_ref[0] = amplitude*np.pi**2/(2*duration**2)*np.cos(np.pi*state.t/duration)
        feedforward = nominal.feedforward(state.q, state.dq, a_ref, config['feedforward'])
        command = JointCommand(q_ref, v_ref, args.gain_scale*np.array(config['kp']),
                               np.array(config['kd']), feedforward, state.t, state.t+config['dt'], 0)
        result = adapter.apply(sim.data, command, state)
        rows.append(np.r_[state.t, state.q, q_ref, state.dq, v_ref, result.tau_applied, result.saturated])
        sim.end_step()
    samples = np.array(rows)
    np.savez_compressed(output, samples=samples)
    from panda_mpc.analyze import plt
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    time = samples[:, 0]
    axes[0].plot(time, samples[:, 1], label='J1 actual')
    axes[0].plot(time, samples[:, 8], '--', label='J1 reference')
    axes[0].set_ylabel('position [rad]'); axes[0].legend()
    error = samples[:, 1:8]-samples[:, 8:15]
    axes[1].plot(time, error); axes[1].set_ylabel('all joint errors [rad]')
    axes[2].plot(time, samples[:, 29:36]); axes[2].set_ylabel('applied torque [Nm]')
    axes[2].set_xlabel('time [s]'); fig.tight_layout()
    fig.savefig(output.with_suffix('.png'), dpi=130); plt.close(fig)
    summary = {'feedforward': args.feedforward, 'kp_scale': args.gain_scale, 'kd_scale': 1.,
               'duration_s': 5., 'final_joint_error_rad': (sim.data.qpos-q_ref).tolist(),
               'joint_error_rms_rad': np.sqrt(np.mean(error**2, axis=0)).tolist(),
               'saturated_steps': int(np.any(samples[:, 36:43], axis=1).sum())}
    write_json(output.with_suffix('.json'), summary)
    print(summary)
    print(f'Data: {output}; plot: {output.with_suffix(".png")}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
