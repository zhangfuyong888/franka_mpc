"""Plant-only external generalized torques, Nm; overwritten and cleared every step."""
import numpy as np


class Disturbance:
    def __init__(self, config):
        self.plan = config['disturbance'].copy()
        self.enabled = config['scenario'] == 'disturbance' and config['disturbance_enabled']

    def apply(self, data, t):
        data.qfrc_applied[:] = 0
        data.xfrc_applied[:] = 0
        p = self.plan
        active = self.enabled and p['start']-1e-10 <= t < p['start']+p['duration']-1e-10
        if active:
            data.qfrc_applied[:] = p['torque']
        return data.qfrc_applied.copy()

    def description(self):
        return {'enabled': self.enabled, 'kind': 'generalized_joint_torque', 'units': 'Nm', **self.plan}
