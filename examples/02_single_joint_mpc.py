"""调用真实 Python OSQP 控制器，单轴理想模型在 2 s 更换目标。"""
import argparse
import numpy as np
from panda_mpc.config import load_config
from panda_mpc.mpc import LinearMPC

parser = argparse.ArgumentParser()
parser.add_argument('--horizon', type=int, default=25)
parser.add_argument('--smoothness', type=float, default=.3)
parser.add_argument('--output', default='single_joint_mpc.npz')
a = parser.parse_args()
c = load_config(horizon=a.horizon, delta_a_weight=a.smoothness)
controller = LinearMPC(np.array([-1.]), np.array([1.]), np.array([.7]), np.array([2.]), c)
q, v, previous = np.zeros(1), np.zeros(1), np.zeros(1)
rows = []
for k in range(400):
    goal = np.array([.6 if k < 100 else -.4])
    result = controller.solve(q, v, goal, previous)
    if not result.success:
        raise RuntimeError(result.status)
    rows.append(np.r_[k*c['period'], q, v, result.a_pred[0], goal])
    q, v, previous = result.q_pred[1], result.dq_pred[1], result.a_pred[0]
np.savez_compressed(a.output, trajectory=np.array(rows))
print('Final error:', np.abs(q-goal), 'Final speed:', np.abs(v), 'Saved:', a.output)
