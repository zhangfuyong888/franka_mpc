"""MuJoCo RGB 离屏渲染 → FFmpeg H.264/yuv420p MP4，按仿真时间采样。"""
import os
from pathlib import Path
import shutil
import subprocess
import sys
from .rendering import overview_camera, SceneOverlay
from .telemetry import write_json


def find_ffmpeg():
    """优先使用显式配置、PATH 或可选 wheel，最后查找当前 Conda 的已有环境。"""
    override = os.environ.get('PANDA_MPC_FFMPEG')
    if override:
        executable = shutil.which(override)
        if executable:
            return executable
        raise RuntimeError(f'PANDA_MPC_FFMPEG is not executable: {override}')
    executable = shutil.which('ffmpeg')
    if executable:
        return executable
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except (ImportError, RuntimeError):
        pass
    prefix = Path(sys.base_prefix)
    candidates = [prefix/'bin'/'ffmpeg']
    if prefix.parent.name == 'envs':
        candidates += [prefix.parent.parent/'bin'/'ffmpeg']
        candidates += sorted(prefix.parent.glob('*/bin/ffmpeg'))
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    raise RuntimeError("H.264 recording requires FFmpeg: python -m pip install -e '.[video]' "
                       "or set PANDA_MPC_FFMPEG to an existing FFmpeg executable")


def h264_options():
    # 8-bit 4:2:0 + Main profile，避免旧播放器对像素格式/Profile 的兼容性问题。
    # faststart 把 MP4 索引置于文件开头，便于播放器及浏览器尽早开始播放。
    return ['-c:v', 'libx264', '-profile:v', 'main', '-pix_fmt', 'yuv420p',
            '-preset', 'fast', '-crf', '20', '-movflags', '+faststart', '-an']


class VideoRecorder:
    def __init__(self, sim, path, config, overlay=None):
        executable = find_ffmpeg()
        self.executable, self.webm = executable, config['video_webm']
        import mujoco
        self.sim, self.path = sim, Path(path)
        self.fps = config['video_fps']
        self.width, self.height = config['video_width'], config['video_height']
        self.camera = overview_camera(sim.model, config)
        self.overlay = overlay if overlay is not None else SceneOverlay()
        self.frames = 0
        self.last_time = None
        self.renderer = self.writer = self.encoder_log = None
        self.closed = False
        self.log_path = self.path.with_suffix('.ffmpeg.log')
        self.camera_settings = {key: config[key] for key in
                                ('camera_lookat', 'camera_distance', 'camera_azimuth', 'camera_elevation')}
        if self.path.exists():
            raise FileExistsError(self.path)
        # 只调整渲染缓冲区，不改变动力学模型参数。
        sim.model.vis.global_.offwidth = max(sim.model.vis.global_.offwidth, self.width)
        sim.model.vis.global_.offheight = max(sim.model.vis.global_.offheight, self.height)
        try:
            self.renderer = mujoco.Renderer(sim.model, height=self.height, width=self.width)
            self.encoder_log = self.log_path.open('wb')
            command = [executable, '-hide_banner', '-loglevel', 'error', '-n',
                       '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                       '-s:v', f'{self.width}x{self.height}', '-r', str(self.fps),
                       '-i', 'pipe:0', *h264_options(), str(self.path)]
            # 编码器在独立进程中工作，不把视频原生库加载进 MuJoCo 进程。
            self.writer = subprocess.Popen(command, stdin=subprocess.PIPE,
                                           stdout=subprocess.DEVNULL, stderr=self.encoder_log)
        except Exception:
            self.close()
            raise

    def capture(self, state, target, prediction, goal_id):
        self.overlay.observe(state, goal_id)
        # 用整数帧号决定下一帧的仿真时间，不依赖墙钟，也不累加浮点周期。
        if state.t+1e-9 < self.frames/self.fps:
            return
        self.renderer.update_scene(self.sim.data, camera=self.camera)
        self.overlay.draw(self.renderer.scene, target, prediction)
        rgb = self.renderer.render()
        try:
            self.writer.stdin.write(rgb.tobytes())
        except BrokenPipeError as exc:
            raise RuntimeError(f'H.264 encoder stopped; see {self.log_path}') from exc
        self.frames += 1
        self.last_time = state.t

    def close(self):
        if self.closed:
            return
        self.closed = True
        code = None
        try:
            if self.writer is not None:
                try:
                    self.writer.stdin.close()  # EOF 后编码剩余帧并写完 MP4 索引。
                except BrokenPipeError:
                    pass
                code = self.writer.wait()
                self.writer = None
        finally:
            try:
                if self.encoder_log is not None:
                    self.encoder_log.close()
                    self.encoder_log = None
            finally:
                if self.renderer is not None:
                    self.renderer.close()
                    self.renderer = None
        write_json(self.path.with_suffix('.json'), {
            'file': self.path.name, 'codec': 'h264', 'encoder': 'libx264',
            'pixel_format': 'yuv420p', 'profile': 'main', 'faststart': True,
            'encoder_exit_code': code, 'fps': self.fps,
            'width': self.width, 'height': self.height, 'frames': self.frames,
            'playback_seconds': self.frames/self.fps, 'last_sample_time': self.last_time,
            'sampling': 'pre-step simulation time; constant playback fps',
            'camera': self.camera_settings,
        })
        if code not in (None, 0):
            detail = self.log_path.read_text(errors='replace')[-4000:]
            raise RuntimeError(f'H.264 encoding failed (exit {code}): {detail}')
        if self.webm and self.frames and code == 0:
            # Ubuntu 基础 GStreamer 通常自带 VP9，可作为缺少 H.264 插件时的副本。
            result = subprocess.run([
                self.executable, '-hide_banner', '-loglevel', 'error', '-n',
                '-i', str(self.path), '-c:v', 'libvpx-vp9', '-pix_fmt', 'yuv420p',
                '-b:v', '0', '-crf', '30', '-deadline', 'realtime', '-cpu-used', '6',
                '-row-mt', '1', '-threads', '4', '-an', str(self.path.with_suffix('.webm')),
            ], capture_output=True, text=True)
            if result.returncode:
                raise RuntimeError(f'WebM encoding failed: {result.stderr[-4000:]}')
