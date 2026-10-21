"""同一空间目标从两个 seed 求解；运行状态不会被 IK 改写。"""
import numpy as np
from panda_mpc.config import load_config
from panda_mpc.sim import Simulation
from panda_mpc.ik import InverseKinematics

c = load_config()
sim = Simulation(c)
ik = InverseKinematics(sim.robot, c)
target = np.array([.48, .14, .50])
for offset in [0., .15]:
    seed = sim.robot.home.copy(); seed[0] += offset
    result = ik.solve(target, seed)
    print(result)
    if result.success:
        print('FK:', sim.robot.fk(result.q_goal), 'seed distance:', np.linalg.norm(result.q_goal-seed))
