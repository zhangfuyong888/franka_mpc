"""2 秒半余弦小幅关节参考，解析速度和加速度，比较前馈/增益。"""
import argparse
import numpy as np
from panda_mpc.config import load_config
from panda_mpc.sim import Simulation
from panda_mpc.dynamics import NominalDynamics
from panda_mpc.mit import MITAdapter
from panda_mpc.types import JointCommand

p = argparse.ArgumentParser()
p.add_argument('--feedforward', choices=['none', 'gravity', 'inverse_dynamics'], default='inverse_dynamics')
p.add_argument('--gain-scale', type=float, default=1.)
a = p.parse_args()
if not 0 < a.gain_scale <= 2:
    p.error('--gain-scale must be in (0, 2]')
c = load_config(feedforward=a.feedforward)
sim = Simulation(c)
nominal = NominalDynamics(c['dt'])
adapter = MITAdapter(sim.model, sim.robot, c['torque'])
rows = []
for tick in range(2500):
    s = sim.begin_step()
    T, amplitude = 2., .1
    qr, vr, ar = sim.robot.home.copy(), np.zeros(7), np.zeros(7)
    qr[0] += amplitude*(1-np.cos(np.pi*min(s.t,T)/T))/2
    if s.t < T:
        vr[0] = amplitude*np.pi/(2*T)*np.sin(np.pi*s.t/T)
        ar[0] = amplitude*np.pi**2/(2*T*T)*np.cos(np.pi*s.t/T)
    ff = nominal.feedforward(s.q, s.dq, ar, c['feedforward'])
    cmd = JointCommand(qr, vr, a.gain_scale*np.array(c['kp']), np.array(c['kd']), ff, s.t, s.t+c['dt'], 0)
    result = adapter.apply(sim.data, cmd, s)
    rows.append(np.r_[s.t, s.q, qr, s.dq, vr, result.tau_applied, result.saturated])
    sim.end_step()
np.savez_compressed(f'mit-{a.feedforward}.npz', samples=np.array(rows))
print('Final joint error [rad]:', np.abs(sim.data.qpos-qr))
