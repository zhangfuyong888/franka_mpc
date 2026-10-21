import importlib.resources
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import hashlib
import numpy as np
import pytest
from panda_mpc.config import load_config
from panda_mpc.types import JointCommand, GoalEvent, RobotState
from panda_mpc.mit import compose
from panda_mpc.goals import GoalManager, read_events
from panda_mpc.reference import ReferenceSegment, ReferenceGenerator, ReferenceFailure
from panda_mpc.mpc import MPCProblem
from panda_mpc.disturbances import Disturbance
from panda_mpc.analyze import recovery_time


def state(t=0., q=None, dq=None):
    return RobotState(t, np.zeros(7) if q is None else q, np.zeros(7) if dq is None else dq,
                      np.zeros(3), np.zeros(3))


def test_mit_units_total_clipping_and_expiry():
    cmd = JointCommand(np.full(7, 0.1), np.full(7, 0.2), np.full(7, 100),
                       np.full(7, 10), np.full(7, 3), 0., 0.1, 2)
    r = compose(cmd, np.zeros(7), np.zeros(7), np.full(7, 12), 0.05)
    np.testing.assert_allclose(r.position_feedback, 10)
    np.testing.assert_allclose(r.velocity_feedback, 2)
    np.testing.assert_allclose(r.tau_raw, 15)
    np.testing.assert_allclose(r.tau_applied, 12)
    assert r.saturated.all()
    expired = compose(cmd, np.zeros(7), np.full(7, 0.5), np.full(7, 12), 0.2)
    np.testing.assert_allclose(expired.tau_applied, -5)
    assert expired.emergency
    assert np.all(expired.feedforward == 0)


def test_invalid_command_and_config():
    with pytest.raises(ValueError):
        JointCommand(np.zeros(6), np.zeros(7), np.ones(7), np.ones(7), np.zeros(7), 0, 1, 1)
    for kwargs in [{'period': 0.021}, {'kp': [1]}, {'torque': [float('nan')]*7}, {'duration': -1}]:
        with pytest.raises(ValueError):
            load_config(**kwargs)


def test_assets_complete_original_hashes_and_torque_xml():
    root = Path(str(importlib.resources.files('panda_mpc').joinpath('assets/panda')))
    original = json.loads((root/'provenance.json').read_text())
    for name, expected in original['files'].items():
        data = (root/'assets'/name).read_bytes()
        assert hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest() == expected, name
    m = ET.parse(root/'panda.xml').getroot()
    assert len(m.findall('.//worldbody//joint')) == 7
    motors = m.findall('./actuator/motor')
    assert len(motors) == 7 and not m.findall('.//general')
    assert not m.findall('tendon') and not m.findall('equality')
    for node in motors:
        assert node.get('gear') == '1'
        assert node.get('ctrlrange') == node.get('forcerange')
    for name, y in [('left_finger', 0.03), ('right_finger', -0.03)]:
        body = m.find(f'.//body[@name="{name}"]')
        assert float(body.get('pos').split()[1]) == y
        assert body.find('inertial') is not None and not body.findall('joint')
    assert m.find('.//site[@name="ee_site"]') is not None
    for mesh in m.findall('./asset/mesh'):
        assert (root/'assets'/mesh.get('file')).exists()


def test_latest_only_and_reached_history_and_replay(tmp_path):
    events = [GoalEvent(4, 0., [0, 0, 0]), GoalEvent(5, 0.013, [1, 0, 0]), GoalEvent(6, 0.014, [2, 0, 0])]
    manager = GoalManager(events)
    first = manager.consume(0)
    first.status, first.reached_at = 'reached', 0.01
    latest = manager.consume(0.02)
    assert latest.event.goal_id == 6
    assert first.status == 'reached' and first.reached_at == 0.01
    assert manager.records[1].status == 'superseded'
    assert manager.records[1].activated_at is None
    assert latest.activated_at == 0.02
    assert manager.consume(0.04) is None
    p = tmp_path/'events.json'
    p.write_text(json.dumps([{'goal_id': 8, 'scheduled_time': 1.1, 'position_world': [0.4, 0, 0.5]}]))
    replay = read_events(p)
    assert replay[0].goal_id == 8
    with pytest.raises(ValueError):
        GoalManager([events[0], events[0]])


def test_reach_requires_position_velocity_and_dwell():
    m = GoalManager([GoalEvent(1, 0, [0, 0, 0])])
    r = m.consume(0); r.q_goal = np.zeros(7)
    m.observe(state(0.))
    fast = state(0.15); fast.v_ee[:] = 0.1
    m.observe(fast)
    m.observe(state(0.2)); m.observe(state(0.39))
    assert r.status == 'active'
    m.observe(state(0.4))
    assert r.status == 'reached'


def test_hermite_derivatives_continuity_and_internal_extrema():
    qs, vs, qe, ve = np.array([0.]), np.array([2.]), np.array([0.]), np.array([-2.])
    segment = ReferenceSegment.hermite(0, 1., qs, vs, qe, ve, 1)
    np.testing.assert_allclose(segment.sample(0)[0], qs)
    np.testing.assert_allclose(segment.sample(0)[1], vs)
    np.testing.assert_allclose(segment.sample(1)[0], qe)
    np.testing.assert_allclose(segment.sample(1)[1], ve)
    lo, hi = segment.extrema()
    assert hi[0, 0] == pytest.approx(0.5)
    # Endpoints are legal but the interior violates position bounds.
    assert not segment.legal(np.array([-0.1]), np.array([0.1]), np.array([3]), np.array([5]))
    for t in [0.17, 0.61]:
        h = 1e-6
        numerical_v = (segment.sample(t+h)[0]-segment.sample(t-h)[0])/(2*h)
        numerical_a = (segment.sample(t+h)[1]-segment.sample(t-h)[1])/(2*h)
        np.testing.assert_allclose(numerical_v, segment.sample(t)[1], atol=1e-8)
        np.testing.assert_allclose(numerical_a, segment.sample(t)[2], atol=1e-8)


def test_reference_bounded_backtrack_and_stop():
    c = load_config()
    g = ReferenceGenerator(state(), -np.ones(7), np.ones(7), np.ones(7), np.ones(7), 0.02)
    old_q, old_v, _ = g.boundary(0.02)
    segment = g.update(0.02, np.ones(7), np.zeros(7), 9)
    assert segment.alpha == 0
    assert segment.goal_id == 9
    np.testing.assert_allclose(segment.sample(0.02)[0], old_q)
    np.testing.assert_allclose(segment.sample(0.02)[1], old_v)
    assert segment.legal(-np.ones(7), np.ones(7), np.ones(7), np.ones(7))
    g.segment = ReferenceSegment.hermite(0., .02, np.full(7, .999), np.ones(7),
                                        np.full(7, 1.019), np.ones(7), 0)
    with pytest.raises(ReferenceFailure):
        g.update(.02, np.ones(7), np.zeros(7), 10, braking=True)


def test_qp_assembly_matches_cost_and_dynamics():
    c = load_config(horizon=3)
    p = MPCProblem(-np.ones(2), np.ones(2), np.ones(2), np.full(2, 2.), c)
    q, v, goal, prev = np.array([.1, -.1]), np.array([.2, 0]), np.array([.4, -.3]), np.array([.3, -.2])
    rng = np.random.default_rng(8)
    inputs = rng.normal(0, 0.2, (p.N, 2))
    states = [np.r_[q, v]]
    for a in inputs:
        states.append(p.Ad@states[-1]+p.Bd@a)
    states = np.array(states)
    z = np.r_[states.ravel(), inputs.ravel()]
    linear, l, u = p.vectors(q, v, goal, prev)
    np.testing.assert_allclose((p.G@z)[:p.neq], l[:p.neq], atol=1e-12)
    assert p.inspect(z, l, u)[0] < 1e-12
    def direct(a, x):
        target = np.r_[goal, np.zeros(2)]
        delta = a-np.vstack([prev, a[:-1]])
        return (np.sum(p.state_weights.reshape(p.N+1, 4)*(x-target)**2)
                +c['a_weight']*np.sum(a*a)+c['delta_a_weight']*np.sum(delta*delta))
    zero = np.zeros_like(z)
    qp_diff = .5*z@(p.P@z)+linear@z
    true_diff = direct(inputs, states)-direct(np.zeros_like(inputs), np.zeros_like(states))
    assert qp_diff == pytest.approx(true_diff)
    # Target and previous accepted input affect different linear blocks.
    c1, _, _ = p.vectors(q, v, -goal, prev)
    assert not np.array_equal(c1[:p.ns], linear[:p.ns])
    np.testing.assert_array_equal(c1[p.ns:], linear[p.ns:])
    c2, _, _ = p.vectors(q, v, goal, np.zeros(2))
    np.testing.assert_allclose(linear[p.ns:p.ns+2]-c2[p.ns:p.ns+2], -2*c['delta_a_weight']*prev)
    p7 = MPCProblem(-np.ones(7), np.ones(7), np.ones(7), np.ones(7), load_config())
    assert p7.nz == 539


def test_external_force_array_assignment_and_clear():
    class Data:
        qfrc_applied = np.zeros(7)
        xfrc_applied = np.ones((3, 6))
    data = Data()
    disturbance = Disturbance(load_config(scenario='disturbance'))
    assert np.all(disturbance.apply(data, 3.9) == 0)
    assert np.linalg.norm(disturbance.apply(data, 4.1)) == 6
    assert np.linalg.norm(disturbance.apply(data, 4.12)) == 6
    assert np.all(data.xfrc_applied == 0)
    assert np.all(disturbance.apply(data, 4.3) == 0)


def test_recovery_time_and_thresholds():
    t = np.arange(0, 2, .01)
    error = np.where(t < 1.2, .1, .001)
    recovered = recovery_time(t, error, np.zeros_like(t), 1., load_config())
    assert recovered == pytest.approx(.4)
    assert recovery_time(t, np.ones_like(t), np.zeros_like(t), 1., load_config()) is None


def test_qp_failure_and_inaccurate_result_rejection():
    from types import SimpleNamespace
    from panda_mpc.mpc import LinearMPC
    c = load_config(horizon=2)
    problem = MPCProblem(-np.ones(1), np.ones(1), np.ones(1), np.ones(1), c)
    controller = LinearMPC.__new__(LinearMPC)
    controller.problem = problem
    # Explicit failure injection, confined to this fault-handling unit test.
    class FailedSolver:
        def update(self, **kwargs):
            pass
        def solve(self, **kwargs):
            return SimpleNamespace(x=None, info=SimpleNamespace(status_val=3, status='primal infeasible',
                          prim_res=1., dual_res=1., iter=10))
    controller.solver = FailedSolver()
    q = np.zeros(1)
    assert not controller.solve(q, q, q, q).success
    class InaccurateSolver(FailedSolver):
        def solve(self, **kwargs):
            return SimpleNamespace(x=np.ones(problem.nz)*100,
                info=SimpleNamespace(status_val=2, status='solved inaccurate', prim_res=.01, dual_res=.01, iter=10))
    controller.solver = InaccurateSolver()
    result = controller.solve(q, q, q, q)
    assert not result.success and result.violation > 1
    assert controller.solve(np.array([2]), q, q, q).status == 'measured_state_outside_operating_bounds'


def test_monitor_persistent_failure_and_emergency_events():
    from panda_mpc.monitor import Monitor
    monitor = Monitor(load_config(max_failures=2), None)
    monitor.failure(0., 'injected_qp_failure', 5)
    assert not monitor.stopped
    monitor.failure(.02, 'injected_qp_failure', 5)
    assert monitor.stopped
    assert [e.kind for e in monitor.events] == ['degraded', 'degraded', 'stop']


def test_headless_runner_source_has_no_graphics_on_import():
    # Pure module import must not create or import GUI objects.
    import subprocess
    import sys
    proc = subprocess.run([sys.executable, '-c',
        "import sys; import panda_mpc.runner; assert 'mujoco.viewer' not in sys.modules; assert 'mujoco.renderer' not in sys.modules"],
        capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr


def test_offline_telemetry_and_analysis_plots(tmp_path):
    """Synthetic telemetry tests the artifact pipeline, not robot performance."""
    from panda_mpc.telemetry import Telemetry
    from panda_mpc.analyze import analyze
    from types import SimpleNamespace
    c = load_config(duration=.02)
    path = tmp_path/'synthetic-analysis-unit-test'
    logger = Telemetry(path, c)
    target = GoalManager([GoalEvent(1, 0., [0, 0, 0])])
    record = target.consume(0.); record.q_goal = np.zeros(7)
    for tick in range(10):
        logger.append(t=tick*.002, q=np.zeros(7), dq=np.zeros(7), p_ee=np.zeros(3),
            v_ee=np.zeros(3), target=np.zeros(3), goal_id=1,
            q_des=np.zeros(7), dq_des=np.zeros(7), ddq_des=np.zeros(7),
            a_ideal=np.zeros(7), actual_acceleration=np.zeros(7),
            tau_ff=np.zeros(7), tau_p=np.zeros(7), tau_d=np.zeros(7),
            tau_raw=np.zeros(7), tau_applied=np.zeros(7), saturated=np.zeros(7, bool),
            inner_elapsed=.0001, bridge_alpha=1., mode='tracking', external_torque=np.zeros(7))
    logger.ticks.append({'t': 0., 'mpc': None, 'alpha': 1., 'q_boundary_jump': 0., 'v_boundary_jump': 0., 'elapsed': .001})
    logger.flush(target, Disturbance(c), SimpleNamespace(events=[], stopped=False), {}, .1)
    result = analyze(path)
    assert result['duration_s'] == pytest.approx(.02)
    assert result['saturated_steps'] == 0
    assert len(list((path/'plots').glob('*.png'))) == 6
    assert (path/'summary.json').exists()
    with pytest.raises(FileExistsError):
        Telemetry(path, c)
