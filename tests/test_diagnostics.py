"""用明确标记的合成日志验证诊断，不把这些数据当成机器人性能证据。"""
import json
from pathlib import Path
import numpy as np
import pytest
from panda_mpc.diagnostics import rejection_counts, inspect_run, compare_runs


def test_rejection_categories_support_old_and_new_logs():
    ticks = [
        {'mpc': {'success': False, 'status': 'measured_state_outside_operating_bounds'}},
        {'mpc': {'success': False, 'status': 'primal infeasible'}},
        {'mpc': {'success': False, 'status': 'solved inaccurate'}},
        {'mpc': {'success': False, 'status': 'anything', 'failure_kind': 'input_rejected'}},
        {'mpc': {'success': True, 'status': 'solved'}, 'predicted_contact': {'sample': 3}},
        {'mpc': None},
    ]
    assert rejection_counts(ticks) == dict(mpc_input_rejections=2, qp_solver_failures=1,
                                          qp_solution_rejections=1, mpc_rejections=4,
                                          predicted_contact_rejections=1)


def synthetic_run(path, position_offset=0.):
    path.mkdir()
    t = np.arange(20)*.002
    zeros = np.zeros((20, 7))
    d = dict(t=t, q=zeros+position_offset, dq=zeros, q_des=zeros, dq_des=zeros,
             ddq_des=zeros, a_ideal=zeros, tau_raw=zeros, tau_applied=zeros,
             saturated=np.zeros((20, 7), bool), goal_id=np.ones(20, int),
             p_ee=np.zeros((20, 3)), target=np.zeros((20, 3)),
             bridge_alpha=np.ones(20), mode=np.full(20, 'tracking'), goal_status=np.full(20, 'active'))
    # 首个事件之前尚无目标，真实日志以 NaN 表示；两次相同空白期应允许比较。
    d['target'][:2] = np.nan
    np.savez_compressed(path/'telemetry.npz', **d)
    ticks = [{'t': 0., 'goal_id': 1, 'status': 'solved', 'alpha': 1.,
              'mpc': {'success': True, 'status': 'solved'}},
             {'t': .02, 'goal_id': 1, 'status': 'braking:input', 'alpha': 1.,
              'mpc': {'success': False, 'status': 'invalid_input'}}]
    (path/'ticks.json').write_text(json.dumps(ticks))
    (path/'events.json').write_text(json.dumps({'monitor': [
        {'t': .02, 'kind': 'degraded', 'reason': 'invalid_input', 'goal_id': 1}]}))


def test_inspection_uses_actual_sample_and_previous_control_tick(tmp_path):
    path = tmp_path/'synthetic'
    synthetic_run(path)
    before = inspect_run(path, .019, joint=2, window=.01)
    assert before['control_tick']['t'] <= before['sample']['t']+1e-9
    after = inspect_run(path, .025, joint=2, window=.01)
    assert after['control_tick']['t'] == .02
    assert after['mpc_quality']['status'] == 'invalid_input'
    assert len(after['nearby_events']) == 1
    assert (path/'plots'/'inspection.png').exists()
    assert (path/'inspection.json').exists()
    with pytest.raises(ValueError):
        inspect_run(path, 99.)
    with pytest.raises(ValueError):
        inspect_run(path, .02, joint=8)


def test_parameter_comparison_and_mismatched_task_rejection(tmp_path):
    a, b = tmp_path/'a', tmp_path/'b'
    synthetic_run(a, position_offset=.01)
    synthetic_run(b)
    result = compare_runs(a, b)
    np.testing.assert_allclose(result['metrics']['current']['joint_tracking_rms_rad'], .01)
    assert result['metrics']['baseline']['ee_error_rms_m'] == 0.
    assert (a/'plots'/'parameter_comparison.png').exists()
    with np.load(b/'telemetry.npz') as data:
        modified = {key: data[key] for key in data.files}
    modified['target'] = np.ones((20, 3))
    np.savez_compressed(b/'telemetry.npz', **modified)
    with pytest.raises(ValueError, match='identical times and targets'):
        compare_runs(a, b)
