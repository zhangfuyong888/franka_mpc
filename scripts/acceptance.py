"""Execute all required real experiments and retain each command/exit code.

Use only after successful installation. Failures do not get relabeled as success.
"""
import json
from pathlib import Path
import subprocess
import sys
from datetime import datetime

root = Path('outputs') / f'acceptance-{datetime.now():%Y%m%d-%H%M%S}'
root.mkdir(parents=True, exist_ok=False)
runs = [('hold',10), ('single_target',10), ('switch',12), ('random',30),
        ('rapid_switch',10), ('invalid_goal',8), ('disturbance',12)]
records = []
for scenario, duration in runs:
    command = [sys.executable, '-m', 'panda_mpc.demo', '--scenario', scenario,
               '--headless', '--duration', str(duration), '--seed', '42',
               '--output', str(root/scenario)]
    code = subprocess.run(command).returncode
    summary_path = root/scenario/'summary.json'
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
    passed = code == (2 if scenario == 'invalid_goal' else 0) and summary.get('execution_completed', False)
    if scenario in ('hold', 'single_target', 'disturbance'):
        passed = passed and summary.get('fixed_goal_acceptance', False)
    records.append({'command': command, 'exit_code': code,
                    'expected_code': 2 if scenario == 'invalid_goal' else 0, 'passed': passed})
control = [sys.executable, '-m', 'panda_mpc.demo', '--scenario', 'disturbance',
           '--headless', '--duration', '12', '--seed', '42', '--no-disturbance',
           '--output', str(root/'control')]
records.append({'command': control, 'exit_code': subprocess.run(control).returncode, 'expected_code': 0, 'passed': False})
records[-1]['passed'] = records[-1]['exit_code'] == 0
if all(r['passed'] for r in records):
    command = [sys.executable, '-m', 'panda_mpc.analyze', '--run-dir', str(root/'disturbance'), '--compare', str(root/'control')]
    analysis_code = subprocess.run(command).returncode
    summary = json.loads((root/'disturbance'/'summary.json').read_text())
    paired_ok = summary.get('paired_comparison', {}).get('offset_exceeds_pre_injection_noise', False)
    recovered = summary.get('disturbance', {}).get('recovery_seconds_after_removal') is not None
    records.append({'command': command, 'exit_code': analysis_code, 'expected_code': 0,
                    'passed': analysis_code == 0 and paired_ok and recovered})
(root/'commands.json').write_text(json.dumps(records, indent=2)+'\n')
print('Evidence:', root)
raise SystemExit(0 if all(r['passed'] for r in records) else 1)
