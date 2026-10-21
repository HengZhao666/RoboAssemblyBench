#!/usr/bin/env bash
set -euo pipefail
mode=${1:-gate}
set +u
source /Embodied_AI/miniconda3/etc/profile.d/conda.sh
conda activate internutopia311
export ISAAC_SIM_ROOT=/home/user/isaacsim-5.1.0
source "$ISAAC_SIM_ROOT/setup_conda_env.sh"
set -u
R=/home/user/beam_integration_20261010_upstream6f57dae
P="$R/beam_assisted_20261015"
cd "$P/assisted_repository"
export INTERNUTOPIA_EXTENSION_PROFILE=ur5e_assembly PYTHONNOUSERSITE=1
export PYTHONPATH="$R/runtime_tools/geometry_deps:$PWD:$P/tools:$R/test_dependencies:${PYTHONPATH:-}"
unset BEAM_EXPLORE_POST_SEAT BEAM_EXPLORE_CONTINUE
export BEAM_COLUMN3_SIDE_CLEARANCE_ROUTE=1
export BEAM_COLUMN3_PICKUP_DEPTH_M=-0.004 BEAM_COLUMN3_TRANSPORT_RAISE_M=0.16
export BEAM_COLUMN3_CARRY_PEAK_FORCE_BAND_N=16,24
export BEAM_COLUMN3_TURN_ANGULAR_SPEED_RAD_S=0.20 BEAM_COLUMN3_TURN_PEAK_FORCE_BAND_N=16,24
export BEAM_LOADED_TARGET_HOLD_SECONDS=0 BEAM_GRIPPER_SHELL_VISUALS=1
python "$P/tools/make_C02_launch_gate.py"
