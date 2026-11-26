#!/usr/bin/env bash
# Engine-free offline check with the production environment (no Isaac app started).
set -eu
set +u
source /Embodied_AI/miniconda3/etc/profile.d/conda.sh
conda activate internutopia311
set -u
R=/home/user/beam_integration_20261010_upstream6f57dae
cd "$R/candidate_repository"
export ISAAC_SIM_ROOT=/home/user/isaacsim-5.1.0
set +u
source "$ISAAC_SIM_ROOT/setup_conda_env.sh"
set -u
export PATH="$R/runtime_tools/bin:/Embodied_AI/miniconda3/envs/internutopia311/bin:/Embodied_AI/miniconda3/bin:$PATH"
export PYTHONPATH="$R/runtime_tools/geometry_deps:$R/candidate_repository:${PYTHONPATH:-}"
export INTERNUTOPIA_EXTENSION_PROFILE=ur5e_assembly GIT_PYTHON_GIT_EXECUTABLE=/Embodied_AI/miniconda3/bin/git
export OMNI_KIT_ACCEPT_EULA=YES PYTHONUNBUFFERED=1 PYTHONNOUSERSITE=1
exec python -B "$@"
