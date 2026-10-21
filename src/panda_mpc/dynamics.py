"""Nominal free-space feedforward; never reads plant external/constraint forces."""
import numpy as np
from .robot import load_model


class NominalDynamics:
    def __init__(self, dt=0.002):
        import mujoco
        self.model = load_model(dt)  # independent model, not the plant object
        self.data = mujoco.MjData(self.model)
        self.mass = np.zeros((7, 7))

    def feedforward(self, q, dq, ddq, mode='inverse_dynamics'):
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
        # Avoid version-dependent mj_fullM argument changes: mj_mulM has a stable
        # matrix-vector interface and includes the model's joint armature.
        inertial = np.zeros(7)
        mujoco.mj_mulM(self.model, self.data, inertial, np.asarray(ddq))
        return inertial + self.data.qfrc_bias - self.data.qfrc_passive
