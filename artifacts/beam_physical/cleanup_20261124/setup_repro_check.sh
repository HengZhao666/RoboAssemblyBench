#!/usr/bin/env bash
# Post-cleanup reproduction check of round 62 (2026-11-24). Not a formal round.
# Copies the round-62 launch chain into its own folder, rebuilds the worker config with the
# production preflight (must reproduce SHA 3bf5d839...), then the caller runs a 420-step smoke whose
# progress rows must equal round 62's own rows.
set -eu
R=/home/user/beam_integration_20261010_upstream6f57dae
B=$R/grasp_repairs_20261012_20rounds
S=$B/round62_v239_unload
D=$B/repro_check_20261124
test ! -e "$D"
mkdir -p "$D"
rsync -a --exclude native_finished.json --exclude smoke_out --exclude 'smoke1.*' --exclude __pycache__ \
  --exclude smoke_review.json --exclude manual_review.json --exclude supervisor.log "$S/" "$D/"
sed -i 's#round62_v239_unload#repro_check_20261124#g' "$D/checked_worker.py" "$D/smoke_run.sh"
grep -c repro_check_20261124 "$D/checked_worker.py" "$D/smoke_run.sh"
python3 - "$D/smoke_review.py" <<'EOF'
import sys
p = sys.argv[1]; s = open(p).read()
for a, b in [("'fabrica_beam_v238_lift04/progress_history.jsonl'", "'fabrica_beam_v239_unload/progress_history.jsonl'"),
             ("prefix_rows_identical_to_round61", "prefix_rows_identical_to_round62"),
             ('"""Review the round-62 launch-chain smoke', '"""Post-cleanup reproduction check (2026-11-24): review the round-62 launch-chain smoke')]:
    assert s.count(a) >= 1, a
    s = s.replace(a, b)
open(p, 'w').write(s)
EOF
mv "$D/production_worker_config.json" "$D/production_worker_config.round62.json"
bash $B/offline_v239_unload/run_offline.sh "$D/tools/preflight.py" > "$D/preflight.log" 2>&1
sha256sum "$D/production_worker_config.json" "$D/production_worker_config.round62.json" "$S/production_worker_config.json"
tail -c 300 "$D/preflight.log"
