"""Observable degradation and bounded-failure stopping; not a safety proof."""
import numpy as np
from .types import MonitorEvent


class Monitor:
    def __init__(self, config, robot):
        self.c, self.robot = config, robot
        self.events = []
        self.consecutive_failures = 0
        self.saturated_since = None
        self.stopped = False

    def emit(self, t, kind, reason, goal_id=-1, **values):
        self.events.append(MonitorEvent(t, kind, reason, goal_id, values))

    def failure(self, t, reason, goal_id):
        self.consecutive_failures += 1
        self.emit(t, 'degraded', reason, goal_id)
        if self.consecutive_failures >= self.c['max_failures']:
            self.stop(t, 'persistent_control_failure', goal_id)

    def stop(self, t, reason, goal_id=-1):
        self.stopped = True
        self.emit(t, 'stop', reason, goal_id)

    def inspect_state(self, state, data):
        if not np.isfinite(np.r_[state.q, state.dq, state.p_ee, state.v_ee]).all():
            self.stop(state.t, 'nonfinite_plant_state')
        ranges = self.robot.m.jnt_range[self.robot.joints]
        if np.any(state.q < ranges[:, 0]) or np.any(state.q > ranges[:, 1]):
            self.stop(state.t, 'physical_joint_limit_exceeded')
        if self.robot.contacts(data, self.c['contact_penetration']):
            self.stop(state.t, 'unallowed_actual_contact')
        if any(w.number > 0 for w in data.warning):
            self.stop(state.t, 'mujoco_warning_possible_auto_reset')

    def inspect_actuation(self, t, result, goal_id):
        if result.emergency:
            self.emit(t, 'emergency_damping', 'expired_or_future_command', goal_id)
        if np.any(result.saturated):
            if self.saturated_since is None:
                self.saturated_since = t
            if t-self.saturated_since >= self.c['saturation_stop']:
                self.stop(t, 'sustained_torque_saturation', goal_id)
        else:
            self.saturated_since = None
