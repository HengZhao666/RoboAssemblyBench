#!/usr/bin/env bash
# Run the accepted Stool controller without replacing the main/Beam controller.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
simple=0
for arg in "$@"; do
  case "$arg" in --help|-h|--prepare|--approve-startup|--stop) simple=1;; esac
done
if (( ! simple )); then
  export ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-${ISAAC_PATH:-}}"
  if [[ ! -f "$ISAAC_SIM_ROOT/setup_conda_env.sh" ]]; then
    echo 'Set ISAAC_SIM_ROOT to your installed Isaac Sim 5.1.0 directory.' >&2
    exit 2
  fi
  set +u
  source "$ISAAC_SIM_ROOT/setup_conda_env.sh"
  set -u
fi
export PYTHONNOUSERSITE=1
exec "${STOOL_PYTHON:-python}" "$ROOT/scripts/stool_assisted/run.py" "$@"
