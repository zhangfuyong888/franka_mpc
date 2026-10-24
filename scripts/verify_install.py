"""在独立 venv 中验证 wheel 安装、依赖、自检、测试及第一课。

默认从正常包索引安装；离线时可显式提供同平台 wheelhouse。不会复用现有 site-packages。
"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import venv
import shutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', help='New verification directory; existing directories are refused')
    parser.add_argument('--offline-wheelhouse', type=Path, help='Explicit local dependency wheels; never implies a PyPI download')
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[1]
    output = Path(args.output or project/'outputs'/f'install-check-{datetime.now():%Y%m%d-%H%M%S-%f}').resolve()
    output.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, PYTHONNOUSERSITE='1', MPLCONFIGDIR=str(output/'matplotlib'))
    env.pop('PYTHONPATH', None)
    commands = []
    def run(command, name, cwd=project):
        with (output/f'{name}.log').open('w') as stream:
            result = subprocess.run([str(x) for x in command], cwd=cwd, env=env,
                                    stdout=stream, stderr=subprocess.STDOUT)
        commands.append({'command': [str(x) for x in command], 'exit_code': result.returncode, 'log': f'{name}.log'})
        if result.returncode:
            raise RuntimeError(f'{name} failed; see {output/name}.log')
    result = {'python': sys.version, 'installation_source': 'local-wheelhouse' if args.offline_wheelhouse else 'package-index',
              'offline_wheelhouse': str(args.offline_wheelhouse) if args.offline_wheelhouse else None,
              'passed': False, 'commands': commands}
    if args.offline_wheelhouse and (args.offline_wheelhouse/'provenance.json').exists():
        shutil.copyfile(args.offline_wheelhouse/'provenance.json', output/'dependency-provenance.json')
        result['dependency_provenance'] = 'dependency-provenance.json'
    try:
        environment = output/'venv'
        venv.EnvBuilder(with_pip=True, system_site_packages=False).create(environment)
        python = environment/'bin'/'python'
        install = [python, '-m', 'pip', 'install', '--disable-pip-version-check', '--retries', '0', '--timeout', '10']
        if sys.version_info[:2] == (3, 10) and sys.platform == 'linux':
            install += ['-c', project/'requirements'/'constraints-py310-linux.txt']
        if args.offline_wheelhouse:
            install += ['--no-index', '--find-links', args.offline_wheelhouse.resolve()]
        # 构建工具也装在这个全新环境中，不借用调用脚本的旧环境。
        run(install+['setuptools>=77', 'wheel'], 'build-dependencies')
        run([python, '-m', 'pip', 'wheel', '--no-deps', '--no-build-isolation',
             '--wheel-dir', output/'wheel', project], 'build', cwd=output)
        wheel = next((output/'wheel').glob('panda_mpc_tracking-*.whl'))
        run(install+[str(wheel)+'[test]'], 'install')
        run([python, '-m', 'pip', 'check'], 'pip-check')
        # 从验证目录运行，确认使用的是安装后的 wheel，而非源码目录或外部环境。
        location_check = '''import importlib, pathlib, sys
root=pathlib.Path(sys.prefix).resolve()
for name in ('panda_mpc','numpy','scipy','mujoco','mink','osqp'):
 p=pathlib.Path(importlib.import_module(name).__file__).resolve()
 assert p.is_relative_to(root), (name,str(p),str(root))
 print(name,p)
'''
        run([python, '-c', location_check], 'package-locations', cwd=output)
        run([python, '-m', 'panda_mpc.doctor'], 'doctor', cwd=output)
        run([python, '-m', 'pytest', '-q', '-W', 'error', '-m', 'not video', project/'tests'], 'tests', cwd=output)
        lesson = output/'first-lesson'
        run([python, '-m', 'panda_mpc.demo', '--config', project/'configs'/'tutorial'/'01_single_target.yaml',
             '--headless', '--output', lesson], 'first-lesson', cwd=output)
        summary = json.loads((lesson/'summary.json').read_text())
        assert summary['execution_completed'] and summary['duration_completed'] and summary['fixed_goal_acceptance']
        result['passed'] = True
    except (OSError, RuntimeError, AssertionError) as exc:
        result['error'] = str(exc)
        print(exc)
    finally:
        (output/'verification.json').write_text(json.dumps(result, indent=2)+'\n')
    print(f'Isolated installation: {"PASS" if result["passed"] else "FAILED"}; evidence: {output}')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
