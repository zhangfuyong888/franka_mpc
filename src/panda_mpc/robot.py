"""Name-based model indexing and isolated FK/contact queries."""
from importlib.resources import files
import numpy as np
from .types import RobotState


def model_path():
    return str(files('panda_mpc').joinpath('assets/panda/scene.xml'))


def load_model(dt=0.002):
    import mujoco
    m = mujoco.MjModel.from_xml_path(model_path())
    m.opt.timestep = dt
    if (m.nq, m.nv, m.nu) != (7, 7, 7):
        raise ValueError('Expected exactly seven joint coordinates and torque motors')
    # Direct motor: fixed gain 1, no bias, no activation dynamics.
    if not (np.all(m.actuator_gainprm[:, 0] == 1)
            and np.all(m.actuator_biasprm == 0)
            and np.all(m.actuator_biastype == 0)
            and np.all(m.actuator_dyntype == 0)
            and np.all(m.actuator_gear[:, 0] == 1)
            and np.all(m.actuator_gear[:, 1:] == 0)):
        raise ValueError('Model contains non-torque actuator semantics')
    return m


class Robot:
    def __init__(self, model, margin=0.08):
        import mujoco
        self.m = model
        self.joints = np.array([model.joint(f'joint{i}').id for i in range(1, 8)])
        self.qadr = model.jnt_qposadr[self.joints]
        self.vadr = model.jnt_dofadr[self.joints]
        self.actuators = np.array([model.actuator(f'motor{i}').id for i in range(1, 8)])
        if not np.array_equal(self.qadr, np.arange(7)) or not np.array_equal(self.vadr, np.arange(7)):
            raise ValueError('This seven-axis formulation requires aligned joint coordinates')
        if not np.array_equal(model.actuator_trnid[self.actuators, 0], self.joints):
            raise ValueError('Actuator transmission joint order mismatch')
        self.site = model.site('ee_site').id
        self.lower = model.jnt_range[self.joints, 0] + margin
        self.upper = model.jnt_range[self.joints, 1] - margin
        self.home = model.key('home').qpos.copy()
        if np.any(self.lower >= self.upper) or np.any(self.home < self.lower) or np.any(self.home > self.upper):
            raise ValueError('Invalid position margin/home')
        self.scratch = mujoco.MjData(model)
        self.base_body = model.body('link0').id

    def state(self, data):
        import mujoco
        jac = np.zeros((3, self.m.nv))
        mujoco.mj_jacSite(self.m, data, jac, None, self.site)
        return RobotState(float(data.time), data.qpos[self.qadr].copy(),
                          data.qvel[self.vadr].copy(), data.site_xpos[self.site].copy(),
                          jac[:, self.vadr] @ data.qvel[self.vadr])

    def fk(self, q):
        import mujoco
        self.scratch.qpos[:] = q
        self.scratch.qvel[:] = 0
        mujoco.mj_forward(self.m, self.scratch)
        return self.scratch.site_xpos[self.site].copy()

    def contacts(self, data, penetration=0.001):
        result = []
        for c in data.contact:
            bodies = self.m.geom_bodyid[[c.geom1, c.geom2]]
            # The fixed base rests on the world floor; this is the only allowed pair.
            if set(bodies.tolist()) <= {0, self.base_body}:
                continue
            if c.dist < -penetration:
                result.append({'geom1': int(c.geom1), 'geom2': int(c.geom2), 'distance': float(c.dist)})
        return result

    def check_prediction(self, q_pred, penetration=0.001):
        import mujoco
        for k, q in enumerate(q_pred):
            self.scratch.qpos[:] = q
            self.scratch.qvel[:] = 0
            mujoco.mj_forward(self.m, self.scratch)
            contacts = self.contacts(self.scratch, penetration)
            if contacts:
                return {'sample': k, 'contacts': contacts}
        return None
