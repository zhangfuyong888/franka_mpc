"""验证仿真时间采样、真实 MP4 编解码及共享历史轨迹；不启动桌面窗口。"""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import numpy as np
import pytest
from panda_mpc.config import load_config
from panda_mpc.rendering import SceneOverlay
from panda_mpc.video import find_ffmpeg


@pytest.mark.parametrize('overrides', [
    {'record_video': 'yes'}, {'video_width': 321}, {'video_height': 0},
    {'video_fps': 0}, {'video_fps': 501}, {'camera_distance': -1},
    {'camera_elevation': -90}, {'camera_lookat': [0, 1]},
])
def test_invalid_video_and_camera_config(overrides):
    with pytest.raises(ValueError):
        load_config(**overrides)


def test_shared_overlay_keeps_four_completed_trails():
    overlay = SceneOverlay()
    for goal_id in range(7):
        overlay.observe(SimpleNamespace(t=goal_id*.01, p_ee=np.array([goal_id, 0., 0.])), goal_id)
    assert len(overlay.history) == 4
    assert [int(t[0][0]) for t in overlay.history] == [2, 3, 4, 5]
    assert overlay.goal_id == 6 and overlay.trail[0][0] == 6
    previous = overlay.trail
    overlay.observe(SimpleNamespace(t=.1, p_ee=np.array([6.1, 0, 0])), 6)
    assert overlay.trail is previous
    assert len(overlay.history) == 4
    assert overlay.trail[-1][0] == 6.1


required = ('mujoco', 'mink', 'osqp')
@pytest.mark.video
@pytest.mark.integration
@pytest.mark.skipif(any(importlib.util.find_spec(n) is None for n in required),
                    reason='Video test requires MuJoCo, mink and OSQP')
def test_real_mp4_duration_and_unchanged_physics(tmp_path):
    # EGL 必须在导入 MuJoCo 前选择，所以用独立进程验证完整 CLI。
    try:
        ffmpeg = find_ffmpeg()
    except RuntimeError as exc:
        pytest.skip(str(exc))
    env = dict(os.environ, MUJOCO_GL='egl')
    config = tmp_path/'record.yaml'
    config.write_text('record_video: true\nvideo_webm: true\nvideo_width: 320\nvideo_height: 180\nvideo_fps: 25\n')
    for record in (True, False):
        output = tmp_path/('video' if record else 'control')
        command = [sys.executable, '-m', 'panda_mpc.demo', '--config', str(config),
                   '--scenario', 'single_target', '--duration', '0.4', '--headless',
                   '--output', str(output)]
        if not record:
            command.append('--no-record-video')
        proc = subprocess.run(command, env=env, text=True, capture_output=True)
        assert proc.returncode == 0, proc.stdout+proc.stderr
    path = tmp_path/'video'
    meta = json.loads((path/'simulation.json').read_text())
    assert meta['frames'] == 10 and meta['fps'] == 25
    assert meta['playback_seconds'] == pytest.approx(.4)
    assert meta['codec'] == 'h264' and meta['pixel_format'] == 'yuv420p'
    assert meta['profile'] == 'main' and meta['encoder_exit_code'] == 0
    decoded = subprocess.run([ffmpeg, '-hide_banner', '-i', str(path/'simulation.mp4'),
                              '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-'], capture_output=True)
    assert decoded.returncode == 0, decoded.stderr.decode(errors='replace')
    assert b'Video: h264 (Main)' in decoded.stderr
    assert b'yuv420p' in decoded.stderr
    assert b'25 fps' in decoded.stderr
    frames = np.frombuffer(decoded.stdout, dtype=np.uint8).reshape(-1, 180, 320, 3)
    assert len(frames) == 10 and all(frame.std() > 5 for frame in frames)
    # faststart 的 moov 索引应位于第一个 mdat 媒体数据块之前。
    data = (path/'simulation.mp4').read_bytes()
    assert 0 < data.index(b'moov') < data.index(b'mdat')
    webm = subprocess.run([ffmpeg, '-hide_banner', '-i', str(path/'simulation.webm'),
                           '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-'], capture_output=True)
    assert webm.returncode == 0, webm.stderr.decode(errors='replace')
    assert b'Video: vp9' in webm.stderr
    assert len(webm.stdout) == 10*180*320*3
    assert not (tmp_path/'control'/'simulation.mp4').exists()
    with np.load(path/'telemetry.npz') as recorded, np.load(tmp_path/'control'/'telemetry.npz') as control:
        for key in ('t', 'q', 'dq', 'goal_id', 'tau_applied'):
            np.testing.assert_array_equal(recorded[key], control[key])
