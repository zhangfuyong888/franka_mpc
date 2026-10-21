"""One independent constrained mink solve per activated discrete target."""
from time import perf_counter
import numpy as np
from .types import IKResult, vector


class InverseKinematics:
    def __init__(self, robot, config):
        import mink
        self.robot, self.c = robot, config
        self.configuration = mink.Configuration(robot.m, q=robot.home)
        self.task = mink.FrameTask('ee_site', 'site', position_cost=1.0,
                                   orientation_cost=config['orientation_cost'],
                                   gain=0.8, lm_damping=1.0)
        self.posture = mink.PostureTask(robot.m, cost=config['posture_cost'])
        self.limits = [
            mink.ConfigurationLimit(robot.m, min_distance_from_limits=config['position_margin']),
            mink.VelocityLimit(robot.m, {f'joint{i+1}': float(v) for i, v in enumerate(config['velocity'])}),
        ]

    def solve(self, position_world, q_seed, orientation_preference=None):
        import mink
        start = perf_counter()
        p, seed = vector(position_world, 3), vector(q_seed)
        it, pe, oe = 0, float('inf'), float('nan')
        if np.any(seed < self.robot.lower) or np.any(seed > self.robot.upper):
            return IKResult(False, None, pe, oe, 0, perf_counter()-start, 'seed_outside_operating_bounds')
        self.configuration.update(seed)
        transform = self.configuration.get_transform_frame_to_world('ee_site', 'site')
        rotation = transform.rotation() if orientation_preference is None else mink.SO3.from_matrix(orientation_preference)
        self.task.set_target(mink.SE3.from_rotation_and_translation(rotation, p))
        self.posture.set_target(seed)
        reason = 'iteration_budget'
        try:
            for it in range(self.c['ik_iterations'] + 1):
                pose = self.configuration.get_transform_frame_to_world('ee_site', 'site')
                pe = float(np.linalg.norm(pose.translation()-p))
                oe = float(np.linalg.norm((pose.rotation().inverse() @ rotation).log()))
                if pe < self.c['ik_tolerance']:
                    q = self.configuration.q
                    legal = (np.isfinite(q).all() and np.all(q >= self.robot.lower-1e-7)
                             and np.all(q <= self.robot.upper+1e-7))
                    collision = self.robot.check_prediction([q], self.c['contact_penetration'])
                    if legal and collision is None:
                        return IKResult(True, q.copy(), pe, oe, it, perf_counter()-start, 'converged')
                    reason = 'endpoint_collision' if collision else 'endpoint_limit'
                    break
                if perf_counter()-start > self.c['ik_budget']:
                    reason = 'wall_clock_budget'
                    break
                if it == self.c['ik_iterations']:
                    break
                vel = mink.solve_ik(self.configuration, [self.task, self.posture],
                                    dt=self.c['ik_dt'], solver='osqp',
                                    damping=self.c['ik_damping'], limits=self.limits,
                                    safety_break=True, eps_abs=1e-7, eps_rel=1e-7,
                                    max_iter=10000, verbose=False)
                if not np.isfinite(vel).all():
                    reason = 'nonfinite_velocity'
                    break
                self.configuration.integrate_inplace(vel, self.c['ik_dt'])
        except mink.NoSolutionFound as exc:
            reason = f'ik_qp_failure:{exc}'
        return IKResult(False, None, pe, oe, it, perf_counter()-start, reason)
