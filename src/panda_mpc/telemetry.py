"""Run artifacts and explicit pre-step row timing; always saved on exceptions."""
from dataclasses import asdict, is_dataclass
from importlib import metadata
from importlib.resources import files
from pathlib import Path
import hashlib
import json
import platform
import sys
import subprocess
from datetime import datetime, timezone
import numpy as np
import yaml


def serial(value):
    if is_dataclass(value):
        return serial(asdict(value))
    if isinstance(value, dict):
        return {str(k): serial(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [serial(v) for v in value]
    if isinstance(value, np.ndarray):
        return serial(value.tolist())
    if isinstance(value, np.generic):
        return serial(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def write_json(path, value):
    Path(path).write_text(json.dumps(serial(value), ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def git_version(directory):
    """只读记录源码提交和未提交标志；wheel 安装或没有 Git 时允许不可用。"""
    try:
        options = dict(cwd=directory, capture_output=True, text=True, timeout=5, check=True)
        commit = subprocess.run(['git', 'rev-parse', 'HEAD'], **options).stdout.strip()
        dirty = bool(subprocess.run(['git', 'status', '--porcelain'], **options).stdout.strip())
        return {'commit': commit, 'dirty': dirty}
    except (OSError, subprocess.SubprocessError):
        return {'commit': None, 'dirty': None}


class Telemetry:
    def __init__(self, output, config):
        self.path = Path(output)
        # Do not overwrite a prior experiment, including on failed runs.
        self.path.mkdir(parents=True, exist_ok=False)
        (self.path/'resolved_config.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
        deps = {}
        for name in ('panda-mpc-tracking', 'mujoco', 'mink', 'osqp', 'qpsolvers', 'numpy', 'scipy', 'matplotlib', 'PyYAML'):
            try:
                deps[name] = metadata.version(name)
            except metadata.PackageNotFoundError:
                deps[name] = 'not-installed'
        pkg = Path(str(files('panda_mpc')))
        source_hash = hashlib.sha256()
        for p in sorted(pkg.glob('*.py')):
            source_hash.update(p.name.encode()+p.read_bytes())
        model = pkg/'assets/panda/model_sha256.json'
        write_json(self.path/'metadata.json', {
            'schema_version': 2, 'git': git_version(pkg), 'command': sys.argv,
            'utc': datetime.now(timezone.utc).isoformat(), 'python': sys.version,
            'platform': platform.platform(), 'dependencies': deps,
            'source_sha256': source_hash.hexdigest(),
            'model_manifest': json.loads(model.read_text()),
            'seed': config['seed'], 'dt': config['dt'], 'period': config['period'],
            'timing': 'time.perf_counter wall clock; all samples pre-step at t; acceleration is previous-step velocity finite difference',
        })
        self.rows, self.ticks = [], []

    def append(self, **row):
        self.rows.append({k: np.asarray(v).copy() for k, v in row.items()})

    def flush(self, goals, disturbance, monitor, sampling, wall_seconds, error=None):
        if self.rows:
            np.savez_compressed(self.path/'telemetry.npz', **{k: np.stack([r[k] for r in self.rows]) for k in self.rows[0]})
        write_json(self.path/'ticks.json', self.ticks)
        write_json(self.path/'events.json', {'targets': goals.records if goals else [],
                   'disturbance': disturbance.description() if disturbance else {},
                   'monitor': monitor.events if monitor else [], 'sampling': sampling})
        write_json(self.path/'execution.json', {'wall_seconds': wall_seconds,
                   'simulated_seconds': float(self.rows[-1]['t']) if self.rows else 0.,
                   'error': error, 'stopped': bool(monitor and monitor.stopped)})
