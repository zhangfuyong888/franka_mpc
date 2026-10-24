"""These tests never replace the physics or solver with mocks."""
import importlib.util
import json
import sys
import numpy as np
import pytest
from panda_mpc.config import load_config

missing = [name for name in ('mujoco', 'mink', 'osqp') if importlib.util.find_spec(name) is None]
pytestmark = [pytest.mark.integration, pytest.mark.skipif(bool(missing), reason='Missing real dependencies: '+', '.join(missing))]


def test_torque_model_and_home_hold():
    import mujoco
    from panda_mpc.sim import Simulation
    from panda_mpc.dynamics import NominalDynamics
    from panda_mpc.mit import MITAdapter
    from panda_mpc.types import JointCommand
    c = load_config()
    sim = Simulation(c)
    m, d = sim.model, sim.data
    assert (m.nq, m.nv, m.nu) == (7, 7, 7)
    d.ctrl[:] = 0
    mujoco.mj_forward(m, d)
    np.testing.assert_allclose(d.qfrc_actuator, 0, atol=1e-12)
    d.ctrl[:] = [1, -2, 3, -4, 5, -6, 7]
    mujoco.mj_forward(m, d)
    np.testing.assert_allclose(d.qfrc_actuator, d.ctrl, atol=1e-12)
    nominal = NominalDynamics(c['dt'])
    adapter = MITAdapter(m, sim.robot, c['torque'])
    for _ in range(1000):
        s = sim.begin_step()
        ff = nominal.feedforward(s.q, s.dq, np.zeros(7))
        command = JointCommand(sim.robot.home, np.zeros(7), c['kp'], c['kd'], ff, s.t, s.t+c['dt'], 0)
        result = adapter.apply(d, command, s)
        assert not result.saturated.any()
        sim.end_step()
    assert np.max(np.abs(d.qpos-sim.robot.home)) < 0.005
    assert np.max(np.abs(d.qvel)) < 0.01


def test_mink_independent_state_and_endpoint():
    from panda_mpc.sim import Simulation
    from panda_mpc.ik import InverseKinematics
    sim = Simulation(load_config())
    before = sim.data.qpos.copy()
    ik = InverseKinematics(sim.robot, load_config())
    target = np.array([.48, .14, .5])
    import warnings
    from qpsolvers.warnings import SparseConversionWarning
    with warnings.catch_warnings():
        warnings.simplefilter('error', SparseConversionWarning)
        r = ik.solve(target, before)
    assert r.success, r
    assert np.linalg.norm(sim.robot.fk(r.q_goal)-target) < .002
    np.testing.assert_array_equal(sim.data.qpos, before)
    invalid = ik.solve([2, 2, 2], before)
    assert not invalid.success and invalid.q_goal is None


def test_real_osqp_switch_and_infeasibility():
    from panda_mpc.mpc import LinearMPC
    c = load_config()
    mpc = LinearMPC(-np.ones(7), np.ones(7), np.full(7, .7), np.full(7, 2.), c)
    q, v, previous = np.zeros(7), np.zeros(7), np.zeros(7)
    for k in range(300):
        target = np.full(7, .5 if k < 80 else -.3)
        r = mpc.solve(q, v, target, previous)
        assert r.success, r.status
        np.testing.assert_allclose(r.q_pred[0], q, atol=1e-5)
        np.testing.assert_allclose(r.dq_pred[0], v, atol=1e-5)
        q, v, previous = r.q_pred[1], r.dq_pred[1], r.a_pred[0]
    assert np.max(np.abs(q+.3)) < .01
    assert np.max(np.abs(v)) < .02
    assert not mpc.solve(np.full(7, 2.), v, target, previous).success
    assert not mpc.solve(np.zeros(7), np.full(7, .700001), target, previous).success
    assert not mpc.solve(np.full(7, .999), np.full(7, .7), target, previous).success


@pytest.mark.parametrize('horizon,smoothness', [(25, .3), (25, 3.), (25, 10.), (15, 3.), (40, 3.)])
def test_motion_margin_prevents_next_step_numerical_bound_rejection(horizon, smoothness):
    """回归真实数值问题：不裁剪实测/预测点，连续执行原先会失败的 400 次滚动优化。"""
    from panda_mpc.mpc import LinearMPC
    c = load_config(horizon=horizon, delta_a_weight=smoothness)
    mpc = LinearMPC(np.array([-1.]), np.array([1.]), np.array([.7]), np.array([2.]), c)
    q, velocity, previous = np.zeros(1), np.zeros(1), np.zeros(1)
    for k in range(400):
        result = mpc.solve(q, velocity, np.array([.6 if k < 100 else -.4]), previous)
        assert result.success, (k, result.status, result.operating_violation)
        np.testing.assert_allclose(result.q_pred[0], q, atol=1e-5)
        np.testing.assert_allclose(result.dq_pred[0], velocity, atol=1e-5)
        assert result.operating_violation <= 1e-10
        assert np.max(np.abs(result.dq_pred)) <= .7+1e-10
        assert np.max(np.abs(result.a_pred)) <= 2.+1e-10
        q, velocity, previous = result.q_pred[1], result.dq_pred[1], result.a_pred[0]
    assert abs(q[0]+.4) < .01


def test_fixed_target_closed_loop_and_headless(tmp_path):
    from panda_mpc.runner import run
    path = tmp_path/'single'
    assert run(load_config(scenario='single_target', duration=10.), path, True) == 0
    summary = json.loads((path/'summary.json').read_text())
    assert summary['targets'][-1]['status'] == 'reached'
    assert summary['final_error_m'] < .01
    assert summary['saturated_steps'] == 0
    assert summary['qp_failures'] == 0
    assert summary['reference_backtracks'] == 0
    assert summary['duration_completed']
    assert summary['mpc_rejections'] == summary['mpc_input_rejections'] == summary['qp_solver_failures'] == 0
    assert (path/'report.md').exists()
    assert summary['max_q_reference_jump'] < 1e-10
    assert summary['max_v_reference_jump'] < 1e-10
    assert 'mujoco.viewer' not in sys.modules


def test_switch_replay_and_no_state_teleport(tmp_path):
    from panda_mpc.runner import run
    c = load_config(scenario='switch', duration=6.)
    path = tmp_path/'switch'
    assert run(c, path, True) == 0
    events = json.loads((path/'events.json').read_text())
    d = np.load(path/'telemetry.npz')
    for record in events['targets']:
        if record['activated_at'] is not None:
            assert record['first_command_at'] == record['activated_at']
            assert record['activated_at']-record['event']['scheduled_time'] <= c['period']+1e-9
    differences = np.abs(np.diff(d['q'], axis=0))
    assert np.max(differences) < .01
    switch_index = np.flatnonzero(d['goal_id'] == 2)[0]
    assert np.linalg.norm(d['dq'][switch_index]) > .01
    replay = tmp_path/'replay'
    assert run(c, replay, True, path/'events.json') == 0
    rd = np.load(replay/'telemetry.npz')
    np.testing.assert_allclose(rd['q'], d['q'], atol=1e-9)


def test_disturbance_pair_real_offset_and_recovery(tmp_path):
    from panda_mpc.runner import run
    from panda_mpc.analyze import analyze
    disturbed, control = tmp_path/'disturbed', tmp_path/'control'
    assert run(load_config(scenario='disturbance', duration=12.), disturbed, True) == 0
    assert run(load_config(scenario='disturbance', duration=12., disturbance_enabled=False), control, True) == 0
    summary = analyze(disturbed, control)
    assert summary['paired_comparison']['offset_exceeds_pre_injection_noise']
    assert summary['disturbance']['recovery_seconds_after_removal'] is not None
    assert summary['final_error_m'] < .01


def test_invalid_goal_rejects_old_goal(tmp_path):
    from panda_mpc.runner import run
    path = tmp_path/'invalid'
    assert run(load_config(scenario='invalid_goal', duration=3.), path, True) == 2
    d = np.load(path/'telemetry.npz')
    after = d['t'] >= 1.
    assert np.all(d['goal_id'][after] == 2)
    assert np.all(d['mode'][after] == 'braking')
    assert np.all(np.isnan(d['q_goal'][after]))


def test_random_rapid_switch_closed_loop_and_replay(tmp_path):
    from panda_mpc.runner import run
    c = load_config(scenario='rapid_switch', duration=10., seed=42)
    path, replay = tmp_path/'rapid', tmp_path/'replay'
    assert run(c, path, True) == 0
    events = json.loads((path/'events.json').read_text())
    records = events['targets'][1:]
    positions = np.array([r['event']['position_world'] for r in records])
    lo, hi = np.asarray(c['rapid_switch_box'])
    assert np.all(positions >= lo) and np.all(positions <= hi)
    assert len(np.unique(positions, axis=0)) == len(records) > 5
    assert events['sampling']['accepted'] == len(records)
    for r in records:
        assert r['activated_at']-r['event']['scheduled_time'] <= c['period']+1e-9
        assert r['first_command_at'] == r['activated_at']
    summary = json.loads((path/'summary.json').read_text())
    assert summary['execution_completed'] and summary['duration_s'] == pytest.approx(10.)
    assert summary['qp_failures'] == summary['saturated_steps'] == 0
    assert summary['max_q_reference_jump'] < 1e-10
    assert summary['max_v_reference_jump'] < 1e-10
    assert run(c, replay, True, path/'events.json') == 0
    with np.load(path/'telemetry.npz') as d, np.load(replay/'telemetry.npz') as rd:
        np.testing.assert_allclose(d['q'], rd['q'], atol=1e-9)
        np.testing.assert_array_equal(d['goal_id'], rd['goal_id'])
        assert np.max(np.abs(np.diff(d['q'], axis=0))) < .01
