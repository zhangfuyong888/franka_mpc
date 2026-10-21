"""The sole owner of actuator ctrl. Feedback gains are Nm/rad and Nm s/rad."""
import numpy as np
from .types import ActuationResult, vector


def compose(command, q, dq, limits, now):
    q, dq, limits = vector(q), vector(dq), vector(limits)
    if np.any(limits <= 0):
        raise ValueError('Torque limits must be positive')
    expired = now > command.valid_until + 1e-10 or now < command.stamp - 1e-10
    if expired:
        # Explicit fault behavior; this is not a C1 reference continuation.
        p = np.zeros(7)
        v = -command.kd * dq
        ff = np.zeros(7)
    else:
        p = command.kp * (command.q_des - q)
        v = command.kd * (command.dq_des - dq)
        ff = command.tau_ff.copy()
    raw = p + v + ff
    applied = np.clip(raw, -limits, limits)
    return ActuationResult(p, v, ff, raw, applied, np.abs(raw) > limits, expired)


class MITAdapter:
    def __init__(self, model, robot, torque_limits):
        self.model, self.robot = model, robot
        self.limits = vector(torque_limits)
        ids = robot.actuators
        allowed = np.minimum(model.actuator_ctrlrange[ids, 1], model.actuator_forcerange[ids, 1])
        if np.any(self.limits > allowed):
            raise ValueError('Configured torque exceeds model limits')

    def apply(self, data, command, state):
        result = compose(command, state.q, state.dq, self.limits, state.t)
        data.ctrl[self.robot.actuators] = result.tau_applied
        return result
