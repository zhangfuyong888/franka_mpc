"""安装后自检：区分控制依赖、环境继承以及可选视频/渲染依赖。"""
import argparse
from importlib import metadata, util
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description='Check the tutorial environment without starting a GUI')
    parser.add_argument('--video', action='store_true', help='Check that FFmpeg supports libx264')
    parser.add_argument('--render', action='store_true', help='Test real offscreen rendering in a subprocess')
    args = parser.parse_args()
    print(f'Python: {sys.version.split()[0]} ({sys.executable})')
    ok = (3, 10) <= sys.version_info[:2] < (3, 13)
    for name in ('numpy', 'scipy', 'mujoco', 'mink', 'osqp', 'qpsolvers', 'PyYAML', 'matplotlib'):
        try:
            print(f'OK {name}=={metadata.version(name)}')
        except metadata.PackageNotFoundError:
            ok = False
            print(f'MISSING {name}; install the project with: python -m pip install -e ".[test]"')
    config = Path(sys.prefix)/'pyvenv.cfg'
    if config.exists() and 'include-system-site-packages = true' in config.read_text().lower():
        print('NOTE: this venv inherits external packages; use a clean venv for installation verification.')
    for name in ('numpy', 'mujoco'):
        spec = util.find_spec(name)
        if spec:
            print(f'{name} source: {spec.origin}')
    if args.video:
        from .video import find_ffmpeg
        try:
            executable = find_ffmpeg()
            proc = subprocess.run([executable, '-hide_banner', '-encoders'], capture_output=True, text=True, timeout=15)
            if proc.returncode or 'libx264' not in proc.stdout:
                raise RuntimeError('FFmpeg does not advertise the libx264 encoder')
            print(f'OK H.264 encoder: {executable}')
        except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
            ok = False
            print(f'VIDEO ERROR: {exc}')
    if args.render:
        code = '''import xml.parsers.expat
import mujoco
from panda_mpc.robot import load_model
m=load_model()
d=mujoco.MjData(m)
mujoco.mj_forward(m,d)
with mujoco.Renderer(m,height=120,width=160) as r:
 r.update_scene(d)
 assert r.render().shape == (120,160,3)
print('OK real offscreen rendering')
'''
        proc = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=30)
        if proc.returncode:
            ok = False
            print('RENDER ERROR: '+proc.stderr[-2000:])
            print('On Linux without a desktop, retry with MUJOCO_GL=egl before the command.')
        else:
            print(proc.stdout.strip())
    print('Environment check: '+('PASS' if ok else 'FAILED'))
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
