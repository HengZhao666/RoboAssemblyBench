#!/usr/bin/env bash
set -euo pipefail
route=${1:?Choose physical or assisted}
run_name=${2:?Choose a new output name}
max_steps=${3:-0}
case "$route" in physical|assisted) ;; *) echo 'Route must be physical or assisted' >&2; exit 2;; esac
[[ "$run_name" =~ ^[a-zA-Z0-9_-]+$ ]] || exit 2
[[ "$max_steps" =~ ^[0-9]+$ ]] || exit 2
R=/home/user/beam_integration_20261010_upstream6f57dae
P="$R/beam_assisted_20261015"
OUT="$P/runs/$run_name"
mkdir -p "$P/runs"
exec 9>"$P/beam_run.lock"
flock -n 9 || { echo 'A Beam route run is already active' >&2; exit 2; }
[[ ! -e "$OUT" ]] || { echo 'Output name already exists; choose a new name' >&2; exit 2; }
mkdir "$OUT"
set +u
source /Embodied_AI/miniconda3/etc/profile.d/conda.sh
conda activate internutopia311
set -u
if [[ "$route" == physical ]]; then
  C="$R/candidate_repository"
  RECIPE=fabrica_beam_ur5e_staged
  WORKER="$R/grasp_repairs_20261012_20rounds/round24_native_release/full_assembly_worker.py"
  export BEAM_EXPLORE_POST_SEAT=1 BEAM_EXPLORE_CONTINUE=1
  export BEAM_ROUTE_MANIFEST="$P/tools/physical_manifest.json"
  unset BEAM_ROUTE_TOOLS_MANIFEST
else
  C="$P/assisted_repository"
  RECIPE=fabrica_beam_ur5e_assisted
  export ISAAC_ASSETS_ROOT="${ISAAC_ASSETS_ROOT:-https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1}"
  WORKER="$P/tools/assisted_worker.py"
  unset BEAM_EXPLORE_POST_SEAT BEAM_EXPLORE_CONTINUE
  export BEAM_ROUTE_MANIFEST="$P/tools/assisted_manifest.json"
  export BEAM_ROUTE_TOOLS_MANIFEST="$P/tools/assisted_tools_manifest.json"
fi
cd "$C"
export BEAM_EXECUTION_ROUTE="$route" BEAM_ROUTE_SOURCE="$C"
export ISAAC_SIM_ROOT=/home/user/isaacsim-5.1.0
export PATH="$R/runtime_tools/bin:/Embodied_AI/miniconda3/envs/internutopia311/bin:/Embodied_AI/miniconda3/bin:$PATH"
export PYTHONPATH="$R/runtime_tools/geometry_deps:$C:${PYTHONPATH:-}"
export BEAM_ENTITY_GEOMETRY_CHECK=1 BEAM_VIDEO_CLOCK_TRACE=1 BEAM_POST_FAILURE_VIDEO_SECONDS=5
export BEAM_GRIPPER_SHELL_VISUALS=1 BEAM_COLUMN3_SIDE_CLEARANCE_ROUTE=1
export BEAM_COLUMN3_PICKUP_DEPTH_M=-0.004 BEAM_COLUMN3_TRANSPORT_RAISE_M=0.16
export BEAM_COLUMN3_CARRY_PEAK_FORCE_BAND_N=16,24 BEAM_COLUMN3_TURN_ANGULAR_SPEED_RAD_S=0.20
export BEAM_COLUMN3_TURN_PEAK_FORCE_BAND_N=16,24
export BEAM_PHYSICS_SOLVER_TYPE=TGS BEAM_EXTERNAL_FORCES_EVERY_ITERATION=0
export BEAM_LOADED_TARGET_HOLD_SECONDS=0
unset BEAM_IDLE_HOLD_ANCHOR
export BEAM_PHYSICS_DIAGNOSTICS_UNFILTERED=1 BEAM_PHYSICS_DIAGNOSTICS=1
export BEAM_PHYSICS_DIAGNOSTICS_STRIDE=8
export BEAM_PHYSICS_DIAGNOSTICS_OBJECTS=fabrica_beam_6,fabrica_beam_3,fabrica_beam_2,fabrica_beam_1,fabrica_beam_0
export BEAM_PHYSICS_DIAGNOSTICS_PHASES='*'
export BEAM_PHYSICS_DIAGNOSTICS_START_STEP=0 BEAM_PHYSICS_DIAGNOSTICS_END_STEP=-1
export BEAM_SUPPORT_CONTACT_TRACE=1
export BEAM_ASSISTED_TRACE_PATH="$OUT/assisted_trace.jsonl"
export BEAM_RELEASE_JOINT_TRACE_PATH="$OUT/release_joint_trace.jsonl"
export BEAM_ENTITY_GEOMETRY_TRACE_PATH="$OUT/entity_geometry_trace.jsonl"
export BEAM_PHYSICS_DIAGNOSTICS_PATH="$OUT/physics_boundary_trace.jsonl"
export BEAM_CONTACT_TRACE_PATH="$OUT/contact_trace.jsonl"
export BEAM_CLOSE_CONTROL_TRACE_PATH="$OUT/close_control_trace.jsonl"
export BEAM_CLOSE_GATE_TRACE_PATH="$OUT/close_gate_trace.jsonl"
export BEAM_MOTION_TRACE_PATH="$OUT/motion_trace.jsonl"
export BEAM_HOLD_TRACE_PATH="$OUT/hold_trace.jsonl"
export BEAM_IDLE_HOLD_TRACE_PATH="$OUT/idle_hold_trace.jsonl"
export BEAM_EXPLORATION_TRACE_PATH="$OUT/exploration_trace.jsonl"
export BEAM_RECENTER_LIFECYCLE_TRACE_PATH="$OUT/recenter_lifecycle_trace.jsonl"
export BEAM_GRASP_SWEEP_TRACE_PATH="$OUT/grasp_sweep_trace.jsonl"
export DUAL_FRANKA_ATTACH_DEBUG_PATH="$OUT/attach_debug.jsonl"
export GIT_PYTHON_GIT_EXECUTABLE=/Embodied_AI/miniconda3/bin/git
export INTERNUTOPIA_EXTENSION_PROFILE=ur5e_assembly VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
export OMNI_KIT_ACCEPT_EULA=YES PYTHONUNBUFFERED=1 PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4
cp "$WORKER" "$OUT/worker_snapshot.py"
cp "$0" "$OUT/launcher_snapshot.sh"
cp "$BEAM_ROUTE_MANIFEST" "$OUT/route_source_manifest.json"
cp "$P/tools/export_geometry.py" "$OUT/geometry_export_snapshot.py"
cp "$P/tools/verify_route_sources.py" "$OUT/source_verifier_snapshot.py"
python "$P/tools/verify_route_sources.py" "$OUT/source_before.json"
if [[ "$route" == assisted ]]; then
  python "$P/tools/check_launch_gate.py" "$P/launch_gates/$run_name.json" "$run_name" "$OUT/launch_gate.json"
  python "$P/tools/claim_C_attempt.py" "$P/launch_gates/$run_name.json"
fi
printf '%s\n' "$$" > "$OUT/runner.pid"
date -Is > "$OUT/started_at"
printf '%s\n' "$route" > "$OUT/execution_route"
args=()
if (( max_steps > 0 )); then args+=(--worker-max-steps "$max_steps"); fi
set +e
guard_pid=''
if [[ "$route" == assisted ]]; then
  python -u "$P/tools/watch_visible_meshes.py" "$run_name" > "$OUT/visible_mesh_guard.log" 2>&1 &
  guard_pid=$!
fi
python -u "$WORKER" --worker-mode collect --worker-recipe "$RECIPE" \
  --worker-scene-profile taoyuan_grscenes_tabletop --worker-results-path "$OUT/collect_results.json" \
  --worker-seeds 0 --worker-layout-seeds 505 --max-trials 1 --start-seed 0 \
  --record-live-video --live-video-frame-stride 8 --live-video-fps 30 --keep-video-frames \
  --output-dir "$OUT" --domain-randomization --skip-episode-steps \
  --runtime-constraint-monitor --constraint-check-stride 8 \
  --assembly-sequence-precheck --stage-trajectory-precheck \
  --stage-precheck-stride 64 --stage-precheck-waypoints 4 --headless "${args[@]}"
code=$?
printf '%s\n' "$code" > "$OUT/exit_code"
date -Is > "$OUT/finished_at"
guard_code=0
if [[ -n "$guard_pid" ]]; then
  wait "$guard_pid"
  guard_code=$?
  printf '%s\n' "$guard_code" > "$OUT/visible_mesh_guard_exit_code"
fi
python "$P/tools/verify_route_sources.py" "$OUT/source_after.json"
verify_code=$?
if (( verify_code != 0 )); then exit "$verify_code"; fi
if (( guard_code != 0 )); then exit "$guard_code"; fi
exit "$code"
