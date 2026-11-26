#!/usr/bin/env bash
# usage: run_smoke.sh NAME TIMEOUT_SECONDS  (traces: close control/gate only)
cd /home/user/work/zhaoheng/stool_physical_20261120
source integration/runtime_env.sh
O=outputs/stool_physical/$1
mkdir -p $O
export BEAM_CLOSE_CONTROL_TRACE_PATH=$PWD/$O/close_control_trace.jsonl
export BEAM_CLOSE_GATE_TRACE_PATH=$PWD/$O/close_gate_trace.jsonl
[ -n "$MOTION" ] && export BEAM_MOTION_TRACE_PATH=$PWD/$O/motion_trace.jsonl
[ -n "$HOLD" ] && export BEAM_HOLD_TRACE_PATH=$PWD/$O/hold_trace.jsonl
[ -n "$HOLDFAIL" ] && export STOOL_HOLD_FAIL_TRACE=$PWD/$O/hold_fail_trace.jsonl
exec /Embodied_AI/miniconda3/envs/internutopia311/bin/python -B integration/plumbers_validate.py \
  --tasks ${TASK:-fabrica_stool_circular_ur5e_physical} --seed 4906 --layout-seed 4906 \
  --output-dir $O/native --worker-timeout-seconds $2 --full-debug-traces \
  --record-live-video --live-video-fps 10 --live-video-frame-stride 24 --live-video-encoder stream
