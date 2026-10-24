"""Finite-duration command line experiments. GUI imports are strictly optional."""
import argparse
import json
from pathlib import Path
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
    p.add_argument('--record-video', action=argparse.BooleanOptionalAction, default=None,
                   help='Record simulation.mp4 in the run directory; --no-record-video disables recording')
    a = p.parse_args()
    c = load_config(a.config, scenario=a.scenario, duration=a.duration, seed=a.seed,
                    feedforward=a.feedforward, record_video=a.record_video)
    if a.no_disturbance:
        c['disturbance_enabled'] = False
    output = a.output or f"outputs/{datetime.now():%Y%m%d-%H%M%S-%f}-{c['scenario']}"
    code = run(c, output, a.headless, a.events)
    summary_file = Path(output)/'summary.json'
    if summary_file.exists():
        summary = json.loads(summary_file.read_text())
        if 'final_error_m' in summary:
            last = summary.get('targets', [])
            status = last[-1]['status'] if last else 'none'
            print(f"Final goal: {status}; EE error: {summary['final_error_m']:.6f} m; "
                  f"MPC rejected: {summary['mpc_rejections']} "
                  f"(input={summary['mpc_input_rejections']}, solver={summary['qp_solver_failures']}, "
                  f"solution={summary['qp_solution_rejections']}); saturated steps: {summary['saturated_steps']}")
        if summary.get('fixed_goal_acceptance') is False:
            print('Target did not satisfy the configured position/speed/dwell criteria; see summary.json.')
    print(f'Artifacts: {output}; exit code: {code}')
    if (Path(output)/'report.md').exists():
        print(f'Report: {Path(output)/"report.md"}')
    if (Path(output)/'simulation.mp4').exists():
        print(f'Video: {Path(output)/"simulation.mp4"}')
    if (Path(output)/'simulation.webm').exists():
        print(f'Video (Ubuntu fallback): {Path(output)/"simulation.webm"}')
    return code


if __name__ == '__main__':
    raise SystemExit(main())
