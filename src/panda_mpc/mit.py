"""MIT 形式关节阻抗执行层：唯一写入仿真 actuator ctrl 的控制模块。

Kp 单位 Nm/rad，Kd 单位 Nm·s/rad；公式见 docs/control_math.md。
"""
import numpy as np
from .types import ActuationResult, vector


def compose(command, q, dq, limits, now):
    """合成位置/速度反馈和前馈；输入七轴参考与实测状态，返回各力矩分量。"""
    q, dq, limits = vector(q), vector(dq), vector(limits)
    if np.any(limits <= 0):
        raise ValueError('Torque limits must be positive')
    expired = now > command.valid_until + 1e-10 or now < command.stamp - 1e-10
    if expired:
        # 过期或来自未来的命令仅施加 -Kd*dq 阻尼，清除位置反馈与前馈。
        # 这是故障处理，不承诺与旧参考 C1 接续。
        p = np.zeros(7)
        v = -command.kd * dq
        ff = np.zeros(7)
    else:
        p = command.kp * (command.q_des - q)
        v = command.kd * (command.dq_des - dq)
        ff = command.tau_ff.copy()
    # 反馈只计算一次，并对反馈+前馈的总力矩限幅；不能各自限幅后直接相加。
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
        # 模型已验证 gear=1、gain=1、无 bias/激活动态，因此 ctrl 在这里是 Nm。
        data.ctrl[self.robot.actuators] = result.tau_applied
        return result
