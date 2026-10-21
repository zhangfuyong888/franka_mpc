"""Plant state evolves only through MuJoCo after the explicit home initialization."""
from .robot import load_model, Robot


class Simulation:
    def __init__(self, config):
        import mujoco
        self.model = load_model(config['dt'])
        self.robot = Robot(self.model, config['position_margin'])
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key('home').id)
        mujoco.mj_forward(self.model, self.data)

    def begin_step(self):
        import mujoco
        mujoco.mj_step1(self.model, self.data)
        return self.robot.state(self.data)

    def end_step(self):
        import mujoco
        mujoco.mj_step2(self.model, self.data)
