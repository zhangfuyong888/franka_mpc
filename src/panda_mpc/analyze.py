"""Offline plots and measured metrics; uses Agg and never loads MuJoCo/OpenGL."""
import argparse
import json
from pathlib import Path
import os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/panda-mpc-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from .telemetry import write_json


def timing(values, budget):
    a = np.asarray(values, float)
    if not len(a):
        return None
    return {'p50_ms': float(np.percentile(a, 50)*1000), 'p95_ms': float(np.percentile(a, 95)*1000),
            'max_ms': float(np.max(a)*1000), 'over_budget': int(np.sum(a > budget))}


def recovery_time(t, error, speed, end, c):
    since = None
    for ti, ei, vi in zip(t, error, speed):
        if ti < end:
            continue
        if ei < c['reach_error'] and vi < c['reach_speed']:
            since = ti if since is None else since
            if ti-since >= c['reach_dwell']-1e-9:
                return float(ti-end)
        else:
            since = None
    return None


def analyze(run_dir, compare=None):
    path = Path(run_dir)
    c = yaml.safe_load((path/'resolved_config.yaml').read_text())
    events = json.loads((path/'events.json').read_text())
    ticks = json.loads((path/'ticks.json').read_text())
    execution = json.loads((path/'execution.json').read_text())
    if not (path/'telemetry.npz').exists():
        write_json(path/'summary.json', {'execution_completed': False, 'acceptance_passed': False, 'execution': execution})
        return {'execution_completed': False, 'acceptance_passed': False}
    with np.load(path/'telemetry.npz') as source:
        d = {k: source[k] for k in source.files}
    t = d['t']
    err = np.linalg.norm(d['p_ee']-d['target'], axis=1)
    speed = np.linalg.norm(d['v_ee'], axis=1)
    qp = [x['mpc'] for x in ticks if x['mpc'] is not None]
    target_results = []
    for record in events['targets']:
        selected = d['goal_id'] == record['event']['goal_id']
        target_results.append({
            'goal_id': record['event']['goal_id'], 'status': record['status'],
            'reached_at': record['reached_at'],
            'scheduled_time': record['event']['scheduled_time'], 'activated_at': record['activated_at'],
            'activation_delay': None if record['activated_at'] is None else record['activated_at']-record['event']['scheduled_time'],
            'first_command_at': record['first_command_at'],
            'final_error_m': float(err[selected][-1]) if selected.any() else None,
            'max_error_m': float(err[selected].max()) if selected.any() else None})
    sim_seconds = float(t[-1]+c['dt'])
    summary = {
        'execution_completed': execution['error'] is None and not execution['stopped'],
        'fixed_goal_acceptance': (bool(target_results) and target_results[-1]['status'] == 'reached'
                and err[-1] < c['reach_error'] and speed[-1] < c['reach_speed'])
                if c['scenario'] in ('hold', 'single_target', 'disturbance') else None,
        'duration_s': sim_seconds, 'wall_seconds': execution['wall_seconds'],
        'simulation_wall_ratio': sim_seconds/execution['wall_seconds'],
        'final_error_m': float(err[-1]), 'max_error_m': float(err.max()),
        'max_ee_speed_m_s': float(speed.max()),
        'joint_speed_peaks_rad_s': np.max(np.abs(d['dq']), axis=0),
        'joint_torque_peaks_Nm': np.max(np.abs(d['tau_applied']), axis=0),
        'saturated_steps': int(np.any(d['saturated'], axis=1).sum()),
        'saturated_joint_samples': int(d['saturated'].sum()),
        'qp_failures': sum(not x['success'] for x in qp),
        'reference_backtracks': sum(x['alpha'] < 1 for x in ticks),
        'max_q_reference_jump': max((x['q_boundary_jump'] for x in ticks), default=0),
        'max_v_reference_jump': max((x['v_boundary_jump'] for x in ticks), default=0),
        'degraded_ticks': sum(x['kind'] == 'degraded' for x in events['monitor']),
        'braking_seconds': float(np.sum(d['mode'] == 'braking')*c['dt']),
        'qp_timing': timing([x['elapsed'] for x in qp], c['period']),
        'control_tick_timing': timing([x['elapsed'] for x in ticks], c['period']),
        'inner_step_timing': timing(d['inner_elapsed'], c['dt']),
        'targets': target_results,
        'actual_acceleration_source': 'backward difference of measured qvel, first sample unavailable',
    }
    dist = events['disturbance']
    if dist.get('enabled'):
        end = dist['start']+dist['duration']
        after = t >= dist['start']
        summary['disturbance'] = {
            'plan': dist, 'max_error_after_injection_m': float(np.max(err[after])) if after.any() else None,
            'recovery_seconds_after_removal': recovery_time(t, err, speed, end, c),
            'final_error_m': float(err[-1])}
    plots = path/'plots'
    plots.mkdir(exist_ok=True)
    def save(fig, name):
        fig.tight_layout()
        fig.savefig(plots/name, dpi=130)
        plt.close(fig)
    def switches(ax):
        for r in events['targets']:
            if r['activated_at'] is not None:
                ax.axvline(r['activated_at'], color='gray', alpha=0.3, lw=0.8)
    fig, ax = plt.subplots(figsize=(9, 3))
    ax.plot(t, err); ax.axhline(c['reach_error'], ls='--', color='gray')
    switches(ax); ax.set(xlabel='time [s]', ylabel='EE position error [m]')
    save(fig, 'error.png')
    fig, axes = plt.subplots(7, 2, figsize=(11, 15), sharex=True)
    limits_path = path/'limits.json'
    limits = json.loads(limits_path.read_text()) if limits_path.exists() else None
    for j in range(7):
        for col, key, refkey, unit in [(0, 'q', 'q_des', 'rad'), (1, 'dq', 'dq_des', 'rad/s')]:
            ax = axes[j, col]
            ax.plot(t, d[key][:, j], label='actual'); ax.plot(t, d[refkey][:, j], '--', label='reference')
            bounds = [limits['lower'][j], limits['upper'][j]] if col == 0 and limits else ([-c['velocity'][j], c['velocity'][j]] if col else [])
            for bound in bounds:
                ax.axhline(bound, color='gray', lw=0.6)
            ax.set_ylabel(f'J{j+1} [{unit}]')
    axes[0, 0].legend(); save(fig, 'joints.png')
    fig, axes = plt.subplots(7, 1, figsize=(10, 12), sharex=True)
    for j, ax in enumerate(axes):
        for key, label in [('a_ideal', 'MPC ideal'), ('ddq_des', 'sent reference'), ('actual_acceleration', 'velocity difference')]:
            ax.plot(t, d[key][:, j], label=label, alpha=0.8)
        ax.set_ylabel(f'J{j+1} [rad/s²]')
    axes[0].legend(); save(fig, 'acceleration.png')
    fig, axes = plt.subplots(7, 1, figsize=(10, 12), sharex=True)
    for j, ax in enumerate(axes):
        ax.plot(t, d['tau_ff'][:, j], label='feedforward')
        ax.plot(t, (d['tau_p']+d['tau_d'])[:, j], label='feedback')
        ax.plot(t, d['tau_raw'][:, j], label='raw', alpha=0.6)
        ax.plot(t, d['tau_applied'][:, j], '--', label='applied')
        for b in [-c['torque'][j], c['torque'][j]]:
            ax.axhline(b, color='gray', lw=0.6)
        ax.set_ylabel(f'J{j+1} [Nm]')
    axes[0].legend(ncol=4); save(fig, 'torques.png')
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    axes[0].plot([x['t'] for x in ticks], [x['elapsed']*1000 for x in ticks], label='whole tick')
    axes[0].plot([x['t'] for x in ticks if x['mpc']], [x['mpc']['elapsed']*1000 for x in ticks if x['mpc']], label='QP wrapper')
    axes[0].axhline(1000*c['period'], color='gray', ls='--'); axes[0].legend(); axes[0].set_ylabel('wall time [ms]')
    axes[1].plot(t, d['bridge_alpha'], label='bridge alpha')
    axes[1].step(t, d['mode'] == 'braking', label='braking'); axes[1].legend()
    save(fig, 'timing_and_fallback.png')
    fig, axes = plt.subplots(2, 1, figsize=(10, 5), sharex=True)
    axes[0].step(t, d['goal_id'], label='latest goal ID'); axes[0].legend()
    axes[1].plot(t, d['external_torque']); axes[1].set(xlabel='time [s]', ylabel='external torque [Nm]')
    save(fig, 'events.png')
    if compare:
        control = Path(compare)
        cd = np.load(control/'telemetry.npz')
        cc = yaml.safe_load((control/'resolved_config.yaml').read_text())
        required_equal = set(c)-{'disturbance_enabled'}
        if any(c[k] != cc[k] for k in required_equal):
            raise ValueError('Paired runs must use identical configuration except disturbance_enabled')
        if cc['disturbance_enabled'] or not dist.get('enabled'):
            raise ValueError('Compare requires disturbed run versus injection-disabled control')
        if not (np.array_equal(t, cd['t']) and np.array_equal(d['target'], cd['target'])):
            raise ValueError('Paired runs require identical times and goal table')
        deviation = np.linalg.norm(d['p_ee']-cd['p_ee'], axis=1)
        control_error = np.linalg.norm(cd['p_ee']-cd['target'], axis=1)
        before = t < dist['start']
        noise = float(np.max(deviation[before])) if before.any() else 0.
        summary['paired_comparison'] = {'maximum_ee_difference_m': float(deviation.max()),
                    'pre_injection_difference_m': noise,
                    'offset_exceeds_pre_injection_noise': bool(deviation.max() > max(1e-6, 10*noise))}
        fig, ax = plt.subplots(figsize=(10, 3))
        ax.plot(t, err, label='disturbed error'); ax.plot(t, control_error, label='control error')
        ax.plot(t, deviation, label='plant difference'); ax.axvspan(dist['start'], dist['start']+dist['duration'], alpha=0.2)
        ax.set(xlabel='time [s]', ylabel='distance [m]'); ax.legend(); save(fig, 'disturbance_comparison.png')
        cd.close()
    write_json(path/'summary.json', summary)
    return summary


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run-dir', required=True)
    p.add_argument('--compare', help='Undisturbed control run directory')
    a = p.parse_args()
    summary = analyze(a.run_dir, a.compare)
    print(json.dumps(summary, default=lambda x: x.tolist(), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
