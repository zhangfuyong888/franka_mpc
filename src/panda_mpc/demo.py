"""Finite-duration command line experiments. GUI imports are strictly optional."""
import argparse
from datetime import datetime
from .config import load_config
from .runner import run


def main():
    p = argparse.ArgumentParser(description='Panda: IK + linear MPC + simulated MIT joint commands')
    p.add_argument('--scenario', choices=['hold', 'single_target', 'switch', 'random', 'rapid_switch', 'invalid_goal', 'disturbance'])
    p.add_argument('--headless', action='store_true')
    p.add_argument('--duration', type=float)
    p.add_argument('--seed', type=int)
    p.add_argument('--config')
    p.add_argument('--output')
    p.add_argument('--events', help='Replay events.json or a JSON event list')
    p.add_argument('--feedforward', choices=['none', 'gravity', 'inverse_dynamics'])
    p.add_argument('--no-disturbance', action='store_true', help='Paired disturbance scenario with injection disabled')
    a = p.parse_args()
    c = load_config(a.config, scenario=a.scenario, duration=a.duration, seed=a.seed, feedforward=a.feedforward)
    if a.no_disturbance:
        c['disturbance_enabled'] = False
    output = a.output or f"outputs/{datetime.now():%Y%m%d-%H%M%S-%f}-{c['scenario']}"
    code = run(c, output, a.headless, a.events)
    print(f'Artifacts: {output}; exit code: {code}')
    return code


if __name__ == '__main__':
    raise SystemExit(main())
