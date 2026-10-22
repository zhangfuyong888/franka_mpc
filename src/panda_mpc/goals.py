"""Deterministic simulated-time events, latest-only activation, immutable history."""
from dataclasses import asdict
from pathlib import Path
import json
import numpy as np
from .types import GoalEvent, GoalRecord


def read_events(path):
    data = json.loads(Path(path).read_text())
    table = data['targets'] if isinstance(data, dict) else data
    # Accept either recorded lifecycles or plain event tables.
    return [GoalEvent(**(row.get('event', row))) for row in table]


def make_events(config, robot, ik):
    scenario, duration = config['scenario'], config['duration']
    home = robot.fk(robot.home)
    table = [GoalEvent(0, 0., home)]
    sampling = {'box': config['random_box'], 'rejected': [], 'accepted': 0}
    a, b, c = np.array([0.48, 0.14, 0.50]), np.array([0.40, -0.18, 0.60]), np.array([0.52, 0.06, 0.42])
    if scenario in ('single_target', 'disturbance'):
        table.append(GoalEvent(1, 0.2, a))
    elif scenario == 'switch':
        table += [GoalEvent(i+1, t, p) for i, (t, p) in enumerate([(0.2, a), (0.7, b), (1.3, c)])]
    elif scenario == 'rapid_switch':
        table += [GoalEvent(i+1, float(t), [a, b, c][i % 3])
                  for i, t in enumerate(np.arange(0.2, duration, 0.3))]
    elif scenario == 'invalid_goal':
        table += [GoalEvent(1, 0.2, a), GoalEvent(2, 1.0, np.array([2., 2., 2.]))]
    elif scenario == 'random':
        position_rng, time_rng = [np.random.default_rng(s) for s in np.random.SeedSequence(config['seed']).spawn(2)]
        t, goal_id = 0.2, 1
        lo, hi = np.asarray(config['random_box'])
        while t < duration:
            for attempt in range(100):
                p = position_rng.uniform(lo, hi)
                result = ik.solve(p, robot.home)
                if result.success:
                    sampling['accepted'] += 1
                    table.append(GoalEvent(goal_id, float(t), p))
                    break
                sampling['rejected'].append({'position': p.tolist(), 'reason': result.reason,
                                             'error': result.position_error})
            else:
                raise RuntimeError('No IK-validated random target in 100 attempts')
            goal_id += 1
            t += float(time_rng.uniform(*config['random_interval']))
    return table, sampling


class GoalManager:
    def __init__(self, events, error=0.01, speed=0.02, dwell=0.2):
        if len({e.goal_id for e in events}) != len(events):
            raise ValueError('Goal IDs must be unique')
        # Input order breaks equal scheduled-time ties; last input row wins.
        self.records = [GoalRecord(e) for _, e in sorted(enumerate(events), key=lambda x: (x[1].scheduled_time, x[0]))]
        self.index, self.active = 0, None
        self.threshold, self.speed, self.dwell = error, speed, dwell
        self.within_since = None

    def consume(self, t):
        """消费本 tick 到期事件，只激活最新一个；保留旧目标已到达的历史。"""
        due = []
        while self.index < len(self.records) and self.records[self.index].event.scheduled_time <= t+1e-10:
            due.append(self.records[self.index])
            self.index += 1
        if not due:
            return None
        if self.active is not None:
            self.active.superseded_at = t
            if self.active.status == 'active':
                self.active.status = 'superseded'
        for old in due[:-1]:
            old.status, old.superseded_at, old.reason = 'superseded', t, 'coalesced_on_tick'
        new = due[-1]
        new.activated_at, new.status = t, 'active'
        self.active, self.within_since = new, None
        return new

    def observe(self, state):
        """用实测末端位置/速度累计驻留时间；reached 后仍继续追踪同一终点。"""
        r = self.active
        if r is None or r.status != 'active' or r.q_goal is None:
            return
        within = (np.linalg.norm(state.p_ee-r.event.position_world) < self.threshold
                  and np.linalg.norm(state.v_ee) < self.speed)
        if not within:
            self.within_since = None
        elif self.within_since is None:
            self.within_since = state.t
        elif state.t-self.within_since >= self.dwell-1e-10:
            r.status, r.reached_at = 'reached', state.t
