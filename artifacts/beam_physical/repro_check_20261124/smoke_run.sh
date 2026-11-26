#!/usr/bin/env bash
# Round 62 (v239_unload). Derived from round61_v238_lift04/smoke_run.sh (batch 62-66, human 2026-11-24).
# Handover unload lift 0.2 mm (BEAM_HANDOVER_UNLOAD_LIFT_M=0.0002): column 2 rises ~0.15 mm, just
# enough to take its 1.8-3.1 N base load off (0.1 mm already reads 0 N) so the compliant right-hand
# hold can yield sideways while part 0 aligns (round 55), without the 0.35-0.95 mm height error of
# rounds 60/61. Completion tolerance max(lift/2, 0.4 mm) exceeds the ~0.255 mm loaded tracking offset.
# Diagnostics restored to round 59 (full contact/physics traces from step 45,000, hold trace on);
# they only write bytes (rounds 59/60 prefixes identical).
set -eu
ulimit -c 0
trap '' HUP
set +u
source /Embodied_AI/miniconda3/etc/profile.d/conda.sh
conda activate internutopia311
set -u
R=/home/user/beam_integration_20261010_upstream6f57dae
TOOLS="$R/grasp_repairs_20261012_20rounds/repro_check_20261124/tools"
OUT="$R/grasp_repairs_20261012_20rounds/repro_check_20261124/smoke_out"
cd "$R/candidate_repository"
export ISAAC_SIM_ROOT=/home/user/isaacsim-5.1.0
set +u
source "$ISAAC_SIM_ROOT/setup_conda_env.sh"
set -u
export PATH="$R/runtime_tools/bin:/Embodied_AI/miniconda3/envs/internutopia311/bin:/Embodied_AI/miniconda3/bin:$PATH"
export PYTHONPATH="$R/runtime_tools/geometry_deps:$R/candidate_repository:${PYTHONPATH:-}"
# ---- behaviour / acceptance (round 53 values) ----
export BEAM_PHYSICAL_ACCEPTANCE_PROFILE=plumbers_staged_v2
export BEAM_COMMON_POSE_WINDOW=1 BEAM_FINAL_POSE_ACCEPTANCE=1
export BEAM_FINAL_POSE_TRACE_PATH="$OUT/final_pose_window_trace.jsonl"
export CUDA_VISIBLE_DEVICES=GPU-e064bda0-b445-2d7f-50d5-c1180deadea2
export BEAM_STREAMING_CAPTURE=1
export BEAM_POST_FAILURE_VIDEO_SECONDS=0
export BEAM_EXPLORE_CONTINUE=0
export BEAM_GRIPPER_SHELL_VISUALS=1
export BEAM_COLUMN3_SIDE_CLEARANCE_ROUTE=1
export BEAM_EMPTY_RETREAT_LOCAL_IK=1 BEAM_COLUMN2_SIDE_CLEARANCE_ROUTE=1
export BEAM_COLUMN3_TRANSPORT_RAISE_M=0.16
export BEAM_COLUMN3_CARRY_PEAK_FORCE_BAND_N=16,24
export GIT_PYTHON_GIT_EXECUTABLE=/Embodied_AI/miniconda3/bin/git
export INTERNUTOPIA_EXTENSION_PROFILE=ur5e_assembly
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
export OMNI_KIT_ACCEPT_EULA=YES PYTHONUNBUFFERED=1 PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4
export BEAM_COLUMN3_PICKUP_DEPTH_M=-0.004
export BEAM_COLUMN3_TURN_ANGULAR_SPEED_RAD_S=0.20
export BEAM_COLUMN3_TURN_PEAK_FORCE_BAND_N=16,24
export BEAM_PHYSICS_SOLVER_TYPE=TGS
export BEAM_EXTERNAL_FORCES_EVERY_ITERATION=0
export BEAM_IDLE_HOLD_ANCHOR=1
export BEAM_LOADED_TARGET_HOLD_SECONDS=0
export BEAM_EXPLORE_POST_SEAT=0
# ---- round 54 change (kept) ----
export BEAM_CAP0_PICKUP_DEPTH_M=-0.0015
# ---- round 55 change (kept) ----
export BEAM_PART0_WINDOW_SPEEDUP=1
# ---- round 56 changes ----
export BEAM_FAST_MOTION=1 BEAM_FINAL_ASSEMBLY_LOAD=1
# ---- round 58 change (kept) ----
export BEAM_PART0_RELEASE_AFTER_CLEAR=1
# ---- round 59 change (kept) ----
export BEAM_RELEASE_WITHOUT_STACK_WINDOW=1
# ---- round 60 change (kept) ----
export BEAM_HANDOVER_UNLOAD_LIFT=1
# ---- round 62 change (lift 0.4 -> 0.2 mm) ----
export BEAM_HANDOVER_UNLOAD_LIFT_M=0.0002
# ---- optimised worker (bit-exact vs compat over 18,000 steps) ----
export BEAM_AB_OUT="$OUT"
export BEAM_OPT_PROBE_TRACE=1 BEAM_OPT_FASTCOPY=1 BEAM_OPT_FRAME_CACHE=1 BEAM_OPT_VELOCITY_CACHE=1
export BEAM_OPT_FULL_TRACE_FROM_STEP=45000 BEAM_OPT_FULL_TRACE_STRIDE=8
# ---- probe/view-creating observers kept (bytes of the entity trace dropped) ----
export BEAM_CONTACT_TRACE_PATH="$OUT/contact_trace.jsonl"
export BEAM_SUPPORT_CONTACT_TRACE=1
export BEAM_ENTITY_GEOMETRY_CHECK=1
export BEAM_ENTITY_GEOMETRY_TRACE_PATH=/dev/null
# ---- inert diagnostics (2026-11-18 bisection) ----
export BEAM_HOLD_TRACE_PATH="$OUT/hold_trace.jsonl"
export BEAM_MOTION_TRACE_PATH="$OUT/motion_trace.jsonl"
export BEAM_CLOSE_CONTROL_TRACE_PATH="$OUT/close_control_trace.jsonl"
export BEAM_CLOSE_GATE_TRACE_PATH="$OUT/close_gate_trace.jsonl"
export BEAM_RELEASE_JOINT_TRACE_PATH="$OUT/release_joint_trace.jsonl"
export BEAM_RECENTER_LIFECYCLE_TRACE_PATH="$OUT/recenter_lifecycle_trace.jsonl"
export BEAM_IDLE_HOLD_TRACE_PATH="$OUT/idle_hold_trace.jsonl"
export BEAM_EXPLORATION_TRACE_PATH="$OUT/exploration_trace.jsonl"
export DUAL_FRANKA_ATTACH_DEBUG_PATH="$OUT/attach_debug.jsonl"
export BEAM_PHYSICS_DIAGNOSTICS_UNFILTERED=1
export BEAM_PHYSICS_DIAGNOSTICS=1
export BEAM_PHYSICS_DIAGNOSTICS_PATH="$OUT/physics_boundary_trace.jsonl"
export BEAM_PHYSICS_DIAGNOSTICS_STRIDE=8
export BEAM_PHYSICS_DIAGNOSTICS_OBJECTS=fabrica_beam_6,fabrica_beam_3,fabrica_beam_2,fabrica_beam_1,fabrica_beam_0
export BEAM_PHYSICS_DIAGNOSTICS_PHASES='*'
export BEAM_PHYSICS_DIAGNOSTICS_START_STEP=45000 BEAM_PHYSICS_DIAGNOSTICS_END_STEP=-1
mkdir -p "$OUT"
test ! -e "$OUT/runner.pid"
test ! -e "$OUT/collect_results.json"
test ! -e "$OUT/exit_code"
python "$TOOLS/verify_source.py" "$OUT/source_before.json"
printf '%s\n' "$$" > "$OUT/runner.pid"
date -Is > "$OUT/started_at"
set +e
python -u "$TOOLS/../smoke_entry.py" \
  --worker-mode collect \
  --worker-recipe fabrica_beam_ur5e_staged \
  --worker-scene-profile taoyuan_grscenes_tabletop \
  --worker-results-path "$OUT/collect_results.json" \
  --worker-seeds 0 --worker-layout-seeds 505 \
  --max-trials 1 --start-seed 0 \
  --record-live-video --live-video-frame-stride 24 --live-video-fps 10 \
  --output-dir "$OUT" --domain-randomization --skip-episode-steps \
  --runtime-constraint-monitor --constraint-check-stride 8 \
  --assembly-sequence-precheck --stage-trajectory-precheck \
  --stage-precheck-stride 64 --stage-precheck-waypoints 4 --headless
code=$?
printf '%s\n' "$code" > "$OUT/exit_code"
date -Is > "$OUT/finished_at"
python "$TOOLS/verify_source.py" "$OUT/source_after.json"
manifest_code=$?
if [ "$manifest_code" -ne 0 ]; then
  exit "$manifest_code"
fi
exit "$code"
