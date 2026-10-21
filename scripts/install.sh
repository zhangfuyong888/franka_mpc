#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
# Select explicitly: PYTHON=python3.10 bash scripts/install.sh if 3.11 unavailable.
"${PYTHON:-python3.11}" -m venv .venv
.venv/bin/python -m pip install -e '.[test]'
.venv/bin/python -m pip check
.venv/bin/python -m pip freeze --all > requirements.lock
.venv/bin/python -m pytest -q -ra
