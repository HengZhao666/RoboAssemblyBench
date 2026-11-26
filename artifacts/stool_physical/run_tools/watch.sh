#!/usr/bin/env bash
# Emit one line per phase change plus terminal/failure lines of a validation log.
L="$1"
until [ -f "$L" ]; do sleep 2; done
tail -n +1 -F "$L" | awk '
/rollout-progress/ {
  for (i = 1; i <= NF; i++) { n = index($i, "="); if (n) v[substr($i, 1, n-1)] = substr($i, n+1) }
  if (v["phase"] != last) { print "step=" v["step"], "phase=" v["phase"], "sps=" v["average_steps_per_second"]; last = v["phase"]; fflush() }
  next
}
/Traceback|rollout interrupted|Simulation App Shutting Down|timeout-failure|success-criteria|\[validation\] (finished|worker|result)/ { print substr($0, 1, 240); fflush() }'
