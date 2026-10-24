"""串起目标、IK、MPC、参考和力矩执行；用整数 tick 协调 2 ms / 20 ms 周期。

流程见 docs/architecture.md；运行中不直接赋值物理 qpos/qvel。
"""
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


def _activate_latest_goal(goals, ik, monitor, state):
    """慢环入口：只激活最新事件，从实测构型求 IK，不重置运动或参考状态。"""
    record = goals.consume(state.t)
    if record is None:
        return None
    outcome = ik.solve(record.event.position_world, state.q)
    record.ik = asdict(outcome)
    if outcome.success:
        record.q_goal = outcome.q_goal
    else:
        record.status, record.reason = 'rejected', outcome.reason
        monitor.emit(state.t, 'target_rejected', outcome.reason, record.event.goal_id)
    return record


def run(config, output, headless=True, events_file=None):
    """执行有限时长实验，保存数据并分析；返回程序退出码，而非单独的到达判据。"""
    logger = Telemetry(output, config)
    goals = disturbance = monitor = viewer = recorder = None
    sampling, error = {}, None
    start = perf_counter()
    exit_code = 0
    try:
        # 先载入标准库 XML 扩展，再载入图形/视频原生库，避免混合 Conda
        # 环境的 Expat 被 EGL 等原生库抢先加载。
        import xml.parsers.expat
        sim = Simulation(config)
        robot, data = sim.robot, sim.data
        write_json(logger.path/'limits.json', {'lower': robot.lower, 'upper': robot.upper,
                   'velocity': config['velocity'], 'acceleration': config['acceleration']})
        ik = InverseKinematics(robot, config)
        mpc = LinearMPC(robot.lower, robot.upper, config['velocity'], config['acceleration'], config)
        # IK/FK 查询和名义动力学使用独立计算数据；只有 sim.data 是实际物理状态。
        nominal = NominalDynamics(config['dt'])
        adapter = MITAdapter(sim.model, robot, config['torque'])
        # 随机场景先生成并从 home 做 IK 预筛；激活时仍会从当时实测状态重新求 IK。
        events, sampling = (read_events(events_file), {'replay': str(events_file)}) if events_file else make_events(config, robot, ik)
        goals = GoalManager(events, config['reach_error'], config['reach_speed'], config['reach_dwell'])
        disturbance = Disturbance(config)
        monitor = Monitor(config, robot)
        state = robot.state(data)
        ref = ReferenceGenerator(state, robot.lower, robot.upper, config['velocity'],
                                 config['acceleration'], config['period'])
        overlay = None
        if config['record_video'] or not headless:
            from .rendering import SceneOverlay
            overlay = SceneOverlay()
        if config['record_video']:
            from .video import VideoRecorder
            recorder = VideoRecorder(sim, logger.path/'simulation.mp4', config, overlay)
        if not headless:
            from .visualizer import Visualizer
            viewer = Visualizer(sim, config, overlay)
        # 默认 20 ms / 2 ms = 10；浮点时间只记录，调度使用整数 tick。
        stride = round(config['period']/config['dt'])
        steps = int(np.floor(config['duration']/config['dt']+1e-9))
        last_v, last_pred, predicted_p = state.dq.copy(), None, []
        prediction_error = np.full(14, np.nan)
        result = None
        a_ideal = np.zeros(7)
        for tick in range(steps):
            inner_start = perf_counter()
            # step1 更新当前派生量，再读位置/速度/末端量；此时尚未推进下一物理步。
            state = sim.begin_step()
            expected_t = tick*config['dt']
            if abs(state.t-expected_t) > 1e-8:
                monitor.stop(state.t, 'time_discontinuity_possible_auto_reset')
            monitor.inspect_state(state, data)
            if monitor.stopped:
                break
            # 慢环：先激活最新目标，再 IK、MPC、接续参考；同步 IK 可能超出墙钟预算。
            if tick % stride == 0:
                tick_start = perf_counter()
                new = _activate_latest_goal(goals, ik, monitor, state)
                active = goals.active
                goal_id = active.event.goal_id if active else -1
                qgoal = active.q_goal if active else None
                if last_pred is not None:
                    prediction_error = np.r_[state.q, state.dq]-last_pred
                # previous_a 是已接受参考边界的加速度，不是实测差分，也非旧理想输入。
                boundary_q, boundary_v, previous_a = ref.boundary(state.t)
                braking, reason, collision = qgoal is None, '', None
                if not braking:
                    # 优化初值必须是实测状态；不能用 boundary_q/v 替代它。
                    result = mpc.solve(state.q, state.dq, qgoal, previous_a)
                    if result.success:
                        # 接触只检查离散预测构型，既不是 QP 约束，也不是连续避障保证。
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
                    # IK 拒绝时不恢复旧目标；QP 失败/预测接触时也从当前参考边界制动。
                    last_pred, predicted_p, a_ideal = None, [], np.full(7, np.nan)
                    result_status = 'braking:'+reason if reason else 'rejected_goal_braking'
                    segment = ref.update(state.t, boundary_q, np.zeros(7), goal_id, braking=True)
                else:
                    result_status = result.status
                    segment = ref.advance(state.t, result.a_pred[0], goal_id)
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
            # 快环：每 2 ms 采样同一解析曲线，再以实测 q/dq 计算前馈和 MIT 反馈。
            qr, vr, ar = ref.segment.sample(state.t)
            ff = nominal.feedforward(state.q, state.dq, ar, config['feedforward'])
            command = JointCommand(qr, vr, config['kp'], config['kd'], ff, state.t,
                                   ref.segment.start+ref.segment.duration, goal_id)
            actuation = adapter.apply(data, command, state)
            if active and active.first_command_at is None:
                active.first_command_at = state.t
            monitor.inspect_actuation(state.t, actuation, goal_id)
            # 外力只写入 plant；控制器仅通过后续实测状态感知其影响。
            external = disturbance.apply(data, state.t)
            goals.observe(state)
            measured_a = (state.dq-last_v)/config['dt'] if tick else np.full(7, np.nan)
            last_v = state.dq.copy()
            inner_elapsed = perf_counter()-inner_control_start
            if tick % stride == 0:
                logger.ticks[-1]['elapsed'] = perf_counter()-tick_start
            # 行数据都是步前 t 的状态和本步命令；实际加速度是 qvel 后向差分。
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
            if recorder is not None:
                # 以步前物理状态生成视频帧；不会回写 qpos/qvel 或改变控制更新频率。
                recorder.capture(state, active.event.position_world if active else None,
                                 predicted_p, goal_id)
            physics_start = perf_counter()
            # ctrl 和外力都写完后，step2 才让物理状态推进 dt。
            sim.end_step()
            logger.rows[-1]['physics_elapsed'] = np.asarray(perf_counter()-physics_start)
            if abs(data.time-(tick+1)*config['dt']) > 1e-8:
                monitor.stop(float(data.time), 'mujoco_step_time_reset')
            if viewer and not viewer.update(state, active.event.position_world if active else None,
                                           predicted_p, goal_id=goal_id):
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
        if recorder is not None:
            try:
                recorder.close()
            except Exception:
                error = error or traceback.format_exc()
                exit_code = 1
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
