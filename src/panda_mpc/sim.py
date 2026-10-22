"""仅初始化时设置 home；其后物理状态只由 MuJoCo 动力学推进。"""
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
        """更新当前运动学等派生量，返回步前测量，留出控制力矩写入时机。"""
        import mujoco
        mujoco.mj_step1(self.model, self.data)
        return self.robot.state(self.data)

    def end_step(self):
        """使用已写入的 ctrl 和外力计算动力学并推进一个物理步。"""
        import mujoco
        mujoco.mj_step2(self.model, self.data)
