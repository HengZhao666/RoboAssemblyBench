#!/usr/bin/env bash
set -eu
set +u
source /Embodied_AI/miniconda3/etc/profile.d/conda.sh
conda activate internutopia311
export ISAAC_SIM_ROOT=/home/user/isaacsim-5.1.0
source "$ISAAC_SIM_ROOT/setup_conda_env.sh"
set -u
R=/home/user/beam_integration_20261010_upstream6f57dae
P="$R/beam_assisted_20261015"
cd "$P/assisted_repository"
export INTERNUTOPIA_EXTENSION_PROFILE=ur5e_assembly
export PYTHONPATH="$R/runtime_tools/geometry_deps:$PWD:$R/test_dependencies:${PYTHONPATH:-}"
export PYTHONNOUSERSITE=1 BEAM_COLUMN3_SIDE_CLEARANCE_ROUTE=0
unset BEAM_EXPLORE_POST_SEAT BEAM_EXPLORE_CONTINUE BEAM_COLUMN3_PICKUP_DEPTH_M BEAM_COLUMN3_TRANSPORT_RAISE_M BEAM_COLUMN3_CARRY_PEAK_FORCE_BAND_N BEAM_COLUMN3_TURN_ANGULAR_SPEED_RAD_S BEAM_COLUMN3_TURN_PEAK_FORCE_BAND_N
if [[ "${1:-}" == --targeted ]]; then
  shift
  python -m pytest --confcutdir=tests/toolkits "$@" -q
  exit
fi
if [[ "${1:-}" == --configuration-report ]]; then
  export BEAM_COLUMN3_SIDE_CLEARANCE_ROUTE=1
  export BEAM_COLUMN3_PICKUP_DEPTH_M=-0.004 BEAM_COLUMN3_TRANSPORT_RAISE_M=0.16
  export BEAM_COLUMN3_CARRY_PEAK_FORCE_BAND_N=16,24 BEAM_COLUMN3_TURN_ANGULAR_SPEED_RAD_S=0.20
  export BEAM_COLUMN3_TURN_PEAK_FORCE_BAND_N=16,24 BEAM_LOADED_TARGET_HOLD_SECONDS=0
  export BEAM_GRIPPER_SHELL_VISUALS=1
  python "$P/tools/emit_configuration_preflight.py" "$2"
  exit
fi
if [[ "${1:-}" == --final-interface-only ]]; then
  python -m pytest --confcutdir=tests/toolkits tests/toolkits/test_beam_assisted_route.py -k final_compiled_predicate -q
  exit
fi
python -m pytest --confcutdir=tests/toolkits tests/toolkits/test_beam_assisted_route.py tests/toolkits/test_beam_assisted_object_servo.py tests/toolkits/test_beam_assisted_preflight.py tests/toolkits/test_beam_assisted_release_state.py tests/toolkits/test_beam_assisted_camera_contract.py tests/toolkits/test_beam_coordinated.py tests/toolkits/test_beam_canonical_integration.py tests/toolkits/test_beam_idle_hold.py tests/toolkits/test_beam_release_joint_branch.py tests/toolkits/test_beam_side_clearance.py tests/toolkits/test_beam_physics_diagnostics.py tests/toolkits/test_beam_physics_contract.py tests/toolkits/test_beam_runtime_integrity.py tests/toolkits/constraint_checking/test_entity_geometry.py -q
