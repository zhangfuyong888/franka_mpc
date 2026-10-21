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
    assert not mpc.solve(np.full(7, .999), np.full(7, .7), target, previous).success


def test_fixed_target_closed_loop_and_headless(tmp_path):
    from panda_mpc.runner import run
    path = tmp_path/'single'
    assert run(load_config(scenario='single_target', duration=10.), path, True) == 0
    summary = json.loads((path/'summary.json').read_text())
    assert summary['targets'][-1]['status'] == 'reached'
    assert summary['final_error_m'] < .01
    assert summary['saturated_steps'] == 0
    assert summary['qp_failures'] == 0
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
