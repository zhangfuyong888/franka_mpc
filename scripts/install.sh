#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
# Creates a clean environment; never reuses or overwrites an existing one.
# Example: PYTHON=python3.10 VENV_DIR=.venv-tutorial bash scripts/install.sh
env_dir="${VENV_DIR:-.venv}"
if [[ -e "$env_dir" ]]; then
  echo "Environment already exists: $env_dir. Choose a new VENV_DIR; existing files are preserved." >&2
  exit 2
fi
"${PYTHON:-python3}" -m venv "$env_dir"
python_bin="$env_dir/bin/python"
"$python_bin" -c 'import sys; assert (3,10) <= sys.version_info[:2] < (3,13), "Requires Python 3.10-3.12"'
constraints=()
if [[ "$("$python_bin" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}:{sys.platform}")')" == "3.10:linux" ]]; then
  constraints=(-c requirements/constraints-py310-linux.txt)
fi
"$python_bin" -m pip install "${constraints[@]}" 'setuptools>=77' wheel
"$python_bin" -m pip install --no-build-isolation "${constraints[@]}" -e '.[test]'
"$python_bin" -m pip check
"$python_bin" -m panda_mpc.doctor
"$python_bin" -m pytest -q -W error -m 'not video'
# Record this actual installation separately; do not overwrite version constraints.
"$python_bin" -m pip freeze --all > "$env_dir/installed-packages.txt"
echo "Ready: source $env_dir/bin/activate"
