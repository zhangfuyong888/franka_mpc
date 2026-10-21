"""Integer-tick multirate loop. Physical state is never assigned during a run."""
from dataclasses import asdict
from time import perf_counter
import traceback
import numpy as np
from .dynamics import NominalDynamics
from .sim import Simulation
from .ik import InverseKinematics
from .mpc import LinearMPC
from .reference import ReferenceGenerator, ReferenceFailure
from .mit import MITAdapter
from .goals import GoalManager, make_events, read_events
from .disturbances import Disturbance
from .monitor import Monitor
from .telemetry import Telemetry, write_json
from .types import JointCommand


def run(config, output, headless=True, events_file=None):
    logger = Telemetry(output, config)
    goals = disturbance = monitor = viewer = None
    sampling, error = {}, None
    start = perf_counter()
    exit_code = 0
    try:
        sim = Simulation(config)
        robot, data = sim.robot, sim.data
        write_json(logger.path/'limits.json', {'lower': robot.lower, 'upper': robot.upper,
                   'velocity': config['velocity'], 'acceleration': config['acceleration']})
        ik = InverseKinematics(robot, config)
        mpc = LinearMPC(robot.lower, robot.upper, config['velocity'], config['acceleration'], config)
        nominal = NominalDynamics(config['dt'])
        adapter = MITAdapter(sim.model, robot, config['torque'])
        events, sampling = (read_events(events_file), {'replay': str(events_file)}) if events_file else make_events(config, robot, ik)
        goals = GoalManager(events, config['reach_error'], config['reach_speed'], config['reach_dwell'])
        disturbance = Disturbance(config)
        monitor = Monitor(config, robot)
        state = robot.state(data)
        ref = ReferenceGenerator(state, robot.lower, robot.upper, config['velocity'],
                                 config['acceleration'], config['period'])
        if not headless:
            from .visualizer import Visualizer
            viewer = Visualizer(sim)
        stride = round(config['period']/config['dt'])
        steps = int(np.floor(config['duration']/config['dt']+1e-9))
        last_v, last_pred, predicted_p = state.dq.copy(), None, []
        prediction_error = np.full(14, np.nan)
        result = None
        a_ideal = np.zeros(7)
        for tick in range(steps):
            inner_start = perf_counter()
            state = sim.begin_step()
            expected_t = tick*config['dt']
            if abs(state.t-expected_t) > 1e-8:
                monitor.stop(state.t, 'time_discontinuity_possible_auto_reset')
            monitor.inspect_state(state, data)
            if monitor.stopped:
                break
            if tick % stride == 0:
                tick_start = perf_counter()
                new = goals.consume(state.t)
                if new:
                    outcome = ik.solve(new.event.position_world, state.q)
                    new.ik = asdict(outcome)
                    if outcome.success:
                        new.q_goal = outcome.q_goal
                    else:
                        new.status, new.reason = 'rejected', outcome.reason
                        monitor.emit(state.t, 'target_rejected', outcome.reason, new.event.goal_id)
                active = goals.active
                goal_id = active.event.goal_id if active else -1
                qgoal = active.q_goal if active else None
                if last_pred is not None:
                    prediction_error = np.r_[state.q, state.dq]-last_pred
                boundary_q, boundary_v, previous_a = ref.boundary(state.t)
                braking, reason, collision = qgoal is None, '', None
                if not braking:
                    result = mpc.solve(state.q, state.dq, qgoal, previous_a)
                    if result.success:
                        collision = robot.check_prediction(result.q_pred, config['contact_penetration'])
                        if collision:
                            braking, reason = True, 'predicted_contact'
                        else:
                            monitor.consecutive_failures = 0
                            a_ideal = result.a_pred[0]
                            last_pred = np.r_[result.q_pred[1], result.dq_pred[1]]
                            predicted_p = [robot.fk(q) for q in result.q_pred]
                    else:
                        braking, reason = True, 'mpc:'+result.status
                    if reason:
                        monitor.failure(state.t, reason, goal_id)
                if braking:
                    last_pred, predicted_p, a_ideal = None, [], np.full(7, np.nan)
                    result_status = 'braking:'+reason if reason else 'rejected_goal_braking'
                    segment = ref.update(state.t, boundary_q, np.zeros(7), goal_id, braking=True)
                else:
                    result_status = result.status
                    segment = ref.update(state.t, result.q_pred[1], result.dq_pred[1], goal_id)
                if segment.alpha < 1:
                    monitor.emit(state.t, 'reference_backtrack', segment.reason, goal_id, alpha=segment.alpha)
                new_q, new_v, _ = segment.sample(state.t)
                tick_elapsed = perf_counter()-tick_start
                logger.ticks.append({
                    't': state.t, 'goal_id': goal_id, 'status': result_status,
                    'mpc': asdict(result) if result is not None and not (qgoal is None) else None,
                    'ik': new.ik if new else None, 'alpha': segment.alpha,
                    'q_boundary_jump': float(np.max(np.abs(new_q-boundary_q))),
                    'v_boundary_jump': float(np.max(np.abs(new_v-boundary_v))),
                    'predicted_contact': collision, 'elapsed': tick_elapsed,
                    'reference_coefficients': segment.coefficients.tolist(),
                })
                if viewer:
                    err = np.linalg.norm(state.p_ee-active.event.position_world) if active else np.nan
                    print(f't={state.t:.2f} goal={goal_id} error={err:.4f} mode={result_status} tick={tick_elapsed*1000:.1f}ms', end='\r', flush=True)
            if monitor.stopped:
                break
            inner_control_start = perf_counter()
            active = goals.active
            goal_id = active.event.goal_id if active else -1
            qr, vr, ar = ref.segment.sample(state.t)
            ff = nominal.feedforward(state.q, state.dq, ar, config['feedforward'])
            command = JointCommand(qr, vr, config['kp'], config['kd'], ff, state.t,
                                   ref.segment.start+ref.segment.duration, goal_id)
            actuation = adapter.apply(data, command, state)
            if active and active.first_command_at is None:
                active.first_command_at = state.t
            monitor.inspect_actuation(state.t, actuation, goal_id)
            external = disturbance.apply(data, state.t)
            goals.observe(state)
            measured_a = (state.dq-last_v)/config['dt'] if tick else np.full(7, np.nan)
            last_v = state.dq.copy()
            inner_elapsed = perf_counter()-inner_control_start
            if tick % stride == 0:
                logger.ticks[-1]['elapsed'] = perf_counter()-tick_start
            logger.append(t=state.t, q=state.q, dq=state.dq, p_ee=state.p_ee, v_ee=state.v_ee,
                goal_id=goal_id, target=active.event.position_world if active else np.full(3, np.nan),
                goal_status=active.status if active else 'none',
                q_goal=active.q_goal if active and active.q_goal is not None else np.full(7, np.nan),
                q_des=qr, dq_des=vr, ddq_des=ar, a_ideal=a_ideal, actual_acceleration=measured_a,
                kp=command.kp, kd=command.kd, tau_ff=ff,
                tau_p=actuation.position_feedback, tau_d=actuation.velocity_feedback,
                tau_raw=actuation.tau_raw, tau_applied=actuation.tau_applied,
                saturated=actuation.saturated, valid_until=command.valid_until,
                bridge_alpha=ref.segment.alpha, mode='braking' if ref.segment.reason == 'braking' else 'tracking',
                prediction_error=prediction_error, external_torque=external,
                inner_elapsed=inner_elapsed, step_control_elapsed=perf_counter()-inner_start,
                physics_elapsed=0., qp_status=logger.ticks[-1]['status'],
                contact_count=data.ncon)
            physics_start = perf_counter()
            sim.end_step()
            logger.rows[-1]['physics_elapsed'] = np.asarray(perf_counter()-physics_start)
            if abs(data.time-(tick+1)*config['dt']) > 1e-8:
                monitor.stop(float(data.time), 'mujoco_step_time_reset')
            if viewer and not viewer.update(state, active.event.position_world if active else None, predicted_p):
                break
        if monitor.stopped:
            exit_code = 1
        elif any(r.status == 'rejected' for r in goals.records):
            exit_code = 2
    except ReferenceFailure:
        error, exit_code = traceback.format_exc(), 1
        if monitor is not None:
            monitor.stop(state.t, 'no_legal_reference_braking_segment', goal_id)
    except KeyboardInterrupt:
        error, exit_code = 'interrupted', 130
    except Exception:
        error, exit_code = traceback.format_exc(), 1
    finally:
        if viewer is not None:
            try:
                viewer.close()
            except Exception:
                error = error or traceback.format_exc()
                exit_code = 1
        logger.flush(goals, disturbance, monitor, sampling, perf_counter()-start, error)
    if error:
        print(error)
    from .analyze import analyze
    analyze(logger.path)
    return exit_code
