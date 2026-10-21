"""导出真实 A→B→C 抢占的数据流，复用项目闭环与分析入口。"""
import argparse
import json
from pathlib import Path
from panda_mpc.config import load_config
from panda_mpc.runner import run

p = argparse.ArgumentParser()
p.add_argument('--output', default='outputs/teaching-switch')
a = p.parse_args()
code = run(load_config(scenario='switch', duration=12.), a.output, headless=True)
for tick in json.loads((Path(a.output)/'ticks.json').read_text()):
    if tick['ik']:
        print({key: tick[key] for key in ['t', 'goal_id', 'status', 'ik', 'alpha', 'q_boundary_jump', 'v_boundary_jump']})
raise SystemExit(code)
