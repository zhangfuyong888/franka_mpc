"""离线教学诊断：区分拒绝原因、定位某一时刻、比较相同任务的两个实验。"""
import json
from pathlib import Path
import numpy as np
from .telemetry import write_json


def rejection_counts(ticks):
    """兼容旧日志：qp_failures 历史字段仍表示全部 MPC 拒绝，新字段细分原因。"""
    counts = dict(mpc_input_rejections=0, qp_solver_failures=0, qp_solution_rejections=0,
                  predicted_contact_rejections=0)
    for tick in ticks:
        if tick.get('predicted_contact') is not None:
            counts['predicted_contact_rejections'] += 1
        result = tick.get('mpc')
        if result is None or result['success']:
            continue
        kind = result.get('failure_kind')
        if not kind:
            status = result.get('status', '').lower()
            if status in ('invalid_input', 'measured_state_outside_operating_bounds'):
                kind = 'input_rejected'
            elif status in ('solved', 'solved inaccurate'):
                kind = 'solution_rejected'
            else:
                kind = 'solver_failure'
        field = {'input_rejected': 'mpc_input_rejections', 'solver_failure': 'qp_solver_failures',
                 'solution_rejected': 'qp_solution_rejections'}.get(kind, 'qp_solver_failures')
        counts[field] += 1
    counts['mpc_rejections'] = sum(counts[k] for k in
                                 ('mpc_input_rejections', 'qp_solver_failures', 'qp_solution_rejections'))
    return counts


def write_report(path, summary):
    """汇总报告可直接在编辑器或 GitHub Markdown 中阅读；图表使用相对链接。"""
    counts = rejection_counts(json.loads((path/'ticks.json').read_text()))
    rows = [
        ('程序正常结束（未触发监测停止）', summary['execution_completed']),
        ('运行满计划时长', summary['duration_completed']),
        ('仿真时长 s', f"{summary['duration_s']:.3f}"),
        ('末目标位置误差 m', f"{summary['final_error_m']:.6f}"),
        ('输入状态拒绝（未调用求解器）', counts['mpc_input_rejections']),
        ('QP 求解器失败', counts['qp_solver_failures']),
        ('QP 结果复核拒绝', counts['qp_solution_rejections']),
        ('预测接触拒绝', counts['predicted_contact_rejections']),
        ('参考回退次数', summary['reference_backtracks']),
        ('显式制动时间 s', f"{summary['braking_seconds']:.3f}"),
        ('力矩饱和步数', summary['saturated_steps']),
    ]
    text = '# 实验分析报告\n\n'
    text += '先看参考与实际是否贴合，再结合事件判断减速来自控制命令还是执行偏差。\n\n'
    text += '| 指标 | 结果 |\n|---|---|\n'
    text += ''.join(f'| {key} | {value} |\n' for key, value in rows)
    text += '\n目标会被替换时，最后一刻的位置误差不是固定目标稳态误差。到达要同时满足位置、速度和驻留条件。\n'
    text += '\n## 目标生命周期\n\n| 目标 | 状态 | 激活 s | 到达 s | 末次误差 m |\n|---|---|---|---|---|\n'
    for goal in summary['targets']:
        def fmt(value):
            return '—' if value is None else f'{value:.4f}'
        text += (f"| {goal['goal_id']} | {goal['status']} | {fmt(goal['activated_at'])} | "
                 f"{fmt(goal['reached_at'])} | {fmt(goal['final_error_m'])} |\n")
    text += '\n## 读图顺序\n\n'
    for name, description in [
        ('joints', '实际与参考的位置/速度：判断执行是否跟上命令。'),
        ('timing_and_fallback', '上图看计算耗时，下图看 alpha 回退及显式制动；alpha=1 也可能在显式制动。'),
        ('error', '相对当前目标的末端误差；灰色竖线是目标切换。'),
        ('acceleration', '理想 MPC 输入、下发参考加速度、实测速度差分。'),
        ('torques', '前馈、反馈、原始和限幅后力矩。'),
        ('events', '目标编号和实验外力。'),
    ]:
        text += f'- [{name}.png](plots/{name}.png)：{description}\n'
    text += '\n详细原因见 events.json；每 20 ms 预测见 ticks.json；每 2 ms 原始记录见 telemetry.npz。\n'
    text += '\n耗时随机器与是否录制变化，不能把低于周期预算的平均耗时当成硬实时保证。\n'
    (path/'report.md').write_text(text)


def inspect_run(run_dir, at, joint=1, window=0.3):
    """选择最近的真实样本与最近的已发生 MPC tick，而非用未来结果解释过去。"""
    if not np.isfinite(at) or not np.isfinite(window) or window <= 0 or not 1 <= joint <= 7:
        raise ValueError('at must be finite; window must be positive; joint must be 1..7')
    path = Path(run_dir)
    (path/'plots').mkdir(exist_ok=True)
    with np.load(path/'telemetry.npz') as source:
        d = {key: source[key] for key in source.files}
    if not d['t'][0] <= at <= d['t'][-1]+1e-9:
        raise ValueError(f"time must be in [{d['t'][0]:.3f}, {d['t'][-1]:.3f}] s")
    i = int(np.argmin(np.abs(d['t']-at)))
    ticks = json.loads((path/'ticks.json').read_text())
    previous = [tick for tick in ticks if tick['t'] <= d['t'][i]+1e-9]
    tick = previous[-1] if previous else None
    events = json.loads((path/'events.json').read_text())
    sample = {key: d[key][i] for key in
              ('t', 'goal_id', 'goal_status', 'q', 'dq', 'q_des', 'dq_des', 'ddq_des',
               'a_ideal', 'tau_raw', 'tau_applied', 'bridge_alpha', 'mode') if key in d}
    result = {
        'requested_time': at, 'sample': sample,
        'control_tick': None if tick is None else {key: tick.get(key) for key in
                                                  ('t', 'goal_id', 'status', 'alpha', 'predicted_contact')},
        'mpc_quality': None if tick is None or tick.get('mpc') is None else {
            key: tick['mpc'].get(key) for key in ('success', 'status', 'failure_kind', 'primal_residual',
                                                'dual_residual', 'violation', 'operating_violation', 'iterations', 'elapsed')},
        'nearby_events': [e for e in events['monitor'] if abs(e['t']-at) <= window],
    }
    write_json(path/'inspection.json', result)
    # 使用 analyze 的无窗口绘图后端，不引入 MuJoCo/OpenGL。
    from .analyze import plt
    selected = np.abs(d['t']-at) <= window
    t, j = d['t'][selected], joint-1
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for ax, actual, reference, unit in [(axes[0], 'q', 'q_des', 'rad'),
                                       (axes[1], 'dq', 'dq_des', 'rad/s')]:
        ax.plot(t, d[actual][selected, j], label='actual')
        ax.plot(t, d[reference][selected, j], '--', label='reference')
        ax.set_ylabel(f'J{joint} [{unit}]'); ax.legend()
    axes[2].plot(t, d['bridge_alpha'][selected], label='alpha')
    axes[2].step(t, d['mode'][selected] == 'braking', label='braking')
    axes[2].legend(); axes[2].set_xlabel('simulation time [s]')
    for ax in axes:
        ax.axvline(d['t'][i], color='black', ls=':', alpha=.5)
    fig.tight_layout(); fig.savefig(path/'plots'/'inspection.png', dpi=130); plt.close(fig)
    return result


def compare_runs(run_dir, other_dir):
    """控制参数对照要求相同采样时间及目标序列，防止把不同任务直接比较。"""
    path, other = Path(run_dir), Path(other_dir)
    (path/'plots').mkdir(exist_ok=True)
    with np.load(path/'telemetry.npz') as a, np.load(other/'telemetry.npz') as b:
        if not (np.array_equal(a['t'], b['t']) and np.array_equal(a['target'], b['target'], equal_nan=True)):
            raise ValueError('Parameter comparison requires identical times and targets; replay the same events file')
        from .analyze import plt
        fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        metrics = {}
        for label, data in [('current', a), ('baseline', b)]:
            error = np.linalg.norm(data['p_ee']-data['target'], axis=1)
            valid = np.isfinite(error)
            if not valid.any():
                plt.close(fig)
                raise ValueError('Parameter comparison requires at least one active finite target')
            tracking = np.sqrt(np.mean((data['q']-data['q_des'])**2, axis=1))
            metrics[label] = {'ee_error_rms_m': float(np.sqrt(np.mean(error[valid]**2))),
                              'joint_tracking_rms_rad': np.sqrt(np.mean((data['q']-data['q_des'])**2, axis=0)),
                              'saturated_steps': int(np.any(data['saturated'], axis=1).sum())}
            axes[0].plot(data['t'], error, label=label)
            axes[1].plot(data['t'], tracking, label=label)
        axes[0].set_ylabel('EE error [m]'); axes[1].set_ylabel('joint tracking RMS [rad]')
        axes[1].set_xlabel('simulation time [s]')
        for ax in axes:
            ax.legend()
        fig.tight_layout(); fig.savefig(path/'plots'/'parameter_comparison.png', dpi=130); plt.close(fig)
    result = {'current_run': str(path), 'baseline_run': str(other), 'metrics': metrics}
    write_json(path/'parameter_comparison.json', result)
    return result
