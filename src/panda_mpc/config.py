"""Validated, packaged configuration. No environment-dependent default paths."""
from importlib.resources import files
from pathlib import Path
import numpy as np
import yaml


def load_config(path=None, **overrides):
    c = yaml.safe_load(files('panda_mpc').joinpath('defaults.yaml').read_text())
    if path:
        user = yaml.safe_load(Path(path).read_text()) or {}
        unknown = set(user) - set(c)
        if unknown:
            raise ValueError(f'Unknown configuration keys: {sorted(unknown)}')
        c.update(user)
    c.update({k: v for k, v in overrides.items() if v is not None})
    for key in ('dt', 'period', 'duration', 'qp_tolerance', 'qp_acceptance',
                'ik_tolerance', 'ik_dt', 'ik_budget', 'reach_error', 'reach_speed',
                'reach_dwell', 'position_margin', 'saturation_stop'):
        if not np.isfinite(c[key]) or c[key] <= 0:
            raise ValueError(f'{key} must be finite and positive')
    ratio = c['period'] / c['dt']
    if ratio < 1 or not np.isclose(ratio, round(ratio), atol=1e-9, rtol=0):
        raise ValueError('period must be an integer multiple of dt')
    for key in ('horizon', 'ik_iterations', 'max_failures'):
        if isinstance(c[key], bool) or not isinstance(c[key], int) or c[key] < 1:
            raise ValueError(f'{key} must be a positive integer')
    for key in ('kp', 'kd', 'torque', 'velocity', 'acceleration'):
        a = np.asarray(c[key], dtype=float)
        if a.shape != (7,) or not np.all(np.isfinite(a)) or np.any(a <= 0):
            raise ValueError(f'{key} requires seven finite positive values')
    for key in ('q_weight', 'v_weight', 'a_weight', 'delta_a_weight',
                'terminal_scale', 'ik_damping', 'orientation_cost', 'posture_cost'):
        if not np.isfinite(c[key]) or c[key] < 0:
            raise ValueError(f'{key} must be finite and nonnegative')
    if c['a_weight'] == 0 or c['terminal_scale'] == 0:
        raise ValueError('a_weight and terminal_scale must be positive')
    if not np.isfinite(c['mpc_motion_margin']) or not 0 <= c['mpc_motion_margin'] < 1:
        raise ValueError('mpc_motion_margin must be finite and in [0, 1)')
    if c['feedforward'] not in ('none', 'gravity', 'inverse_dynamics'):
        raise ValueError('Unknown feedforward mode')
    if c['scenario'] not in ('hold', 'single_target', 'switch', 'random',
                              'rapid_switch', 'invalid_goal', 'disturbance'):
        raise ValueError('Unknown scenario')
    for prefix in ('random', 'rapid_switch'):
        box = np.asarray(c[prefix+'_box'], float)
        interval = np.asarray(c[prefix+'_interval'], float)
        if box.shape != (2, 3) or not np.isfinite(box).all() or np.any(box[0] >= box[1]):
            raise ValueError(f'{prefix}_box must contain ordered finite xyz corners')
        if interval.shape != (2,) or not np.isfinite(interval).all() or not 0 < interval[0] <= interval[1]:
            raise ValueError(f'{prefix}_interval must be positive and ordered')
    if not isinstance(c['record_video'], bool):
        raise ValueError('record_video must be a boolean')
    if not isinstance(c['video_webm'], bool):
        raise ValueError('video_webm must be a boolean')
    for key in ('video_width', 'video_height'):
        if isinstance(c[key], bool) or not isinstance(c[key], int) or c[key] < 2 or c[key] % 2:
            raise ValueError(f'{key} must be a positive even integer')
    if not np.isfinite(c['video_fps']) or not 0 < c['video_fps'] <= 1/c['dt']:
        raise ValueError('video_fps must be positive and no greater than the physics step rate')
    if not np.isfinite(c['camera_distance']) or c['camera_distance'] <= 0:
        raise ValueError('camera_distance must be finite and positive')
    if not np.isfinite([c['camera_azimuth'], c['camera_elevation']]).all():
        raise ValueError('camera angles must be finite')
    if not -90 < c['camera_elevation'] < 90:
        raise ValueError('camera_elevation must be between -90 and 90 degrees')
    if np.shape(c['camera_lookat']) != (3,) or not np.isfinite(c['camera_lookat']).all():
        raise ValueError('camera_lookat requires three finite coordinates')
    d = c['disturbance']
    if not np.isfinite([d['start'], d['duration']]).all() or d['start'] < 0 or d['duration'] <= 0:
        raise ValueError('invalid disturbance timing')
    if np.shape(d['torque']) != (7,) or not np.isfinite(d['torque']).all():
        raise ValueError('disturbance torque must be finite shape (7,)')
    return c
