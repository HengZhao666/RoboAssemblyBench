#!/usr/bin/env bash
# Public Beam assisted entrypoint. Requires the user's installed Isaac Sim 5.1.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
args=("$@")
simple=0
for arg in "${args[@]}"; do
  case "$arg" in --help|-h|--approve-startup|--stop) simple=1;; esac
done
for ((i=0; i<${#args[@]}; i++)); do
  case "${args[i]}" in
    --isaac-sim-root) export ISAAC_SIM_ROOT="${args[i+1]:?--isaac-sim-root needs a directory}";;
    --isaac-sim-root=*) export ISAAC_SIM_ROOT="${args[i]#*=}";;
  esac
done
if (( ! simple )); then
  export ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-${ISAAC_PATH:-}}"
  if [[ ! -f "$ISAAC_SIM_ROOT/setup_conda_env.sh" ]]; then
    echo 'Set ISAAC_SIM_ROOT to your Isaac Sim 5.1.0 directory, or pass --isaac-sim-root DIR.' >&2
    exit 2
  fi
  set +u
  source "$ISAAC_SIM_ROOT/setup_conda_env.sh"
  set -u
fi
export PYTHONNOUSERSITE=1
export PYTHONPATH="$ROOT${BEAM_EXTRA_PYTHONPATH:+:$BEAM_EXTRA_PYTHONPATH}${PYTHONPATH:+:$PYTHONPATH}"
cd "$ROOT"
exec "${BEAM_PYTHON:-python}" "$ROOT/scripts/beam_assisted/run.py" "${args[@]}"
