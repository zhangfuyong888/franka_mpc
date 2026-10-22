"""独立名义模型计算自由空间前馈，不读取仿真外力或接触约束力。"""
import numpy as np
from .robot import load_model


class NominalDynamics:
    def __init__(self, dt=0.002):
        import mujoco
        self.model = load_model(dt)  # 独立模型和数据，零速度重力查询不会改变真实机器人
        self.data = mujoco.MjData(self.model)
        self.mass = np.zeros((7, 7))

    def feedforward(self, q, dq, ddq, mode='inverse_dynamics'):
        """以实测 q/dq 和参考 ddq 计算七轴力矩 (Nm)，见 docs/control_math.md。"""
        import mujoco
        if mode == 'none':
            return np.zeros(7)
        if mode not in ('gravity', 'inverse_dynamics'):
            raise ValueError(mode)
        self.data.qpos[:] = q
        self.data.qvel[:] = 0 if mode == 'gravity' else dq
        mujoco.mj_forward(self.model, self.data)
        if mode == 'gravity':
            return self.data.qfrc_bias.copy()
        # mj_mulM 直接计算 M(q)*a_ref（含 armature），避免显式展开质量矩阵。
        inertial = np.zeros(7)
        mujoco.mj_mulM(self.model, self.data, inertial, np.asarray(ddq))
        # MuJoCo 中 M*a + bias = tau + passive（此处忽略外力/约束力），
        # 所以所需主动前馈为 M*a_ref + bias - passive；被动力必须取减号。
        return inertial + self.data.qfrc_bias - self.data.qfrc_passive
