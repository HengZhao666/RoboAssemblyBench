#!/usr/bin/env bash
# Offline python inside the stool physical tree with the Plumbers runtime env.
cd /home/user/work/zhaoheng/stool_physical_20261120
source integration/runtime_env.sh >/dev/null 2>&1
exec /Embodied_AI/miniconda3/envs/internutopia311/bin/python -B "$@"
