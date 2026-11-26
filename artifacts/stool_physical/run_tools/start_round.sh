#!/usr/bin/env bash
cd /home/user/work/zhaoheng/stool_physical_20261120
source integration/runtime_env.sh
exec /Embodied_AI/miniconda3/envs/internutopia311/bin/python -B outputs/stool_physical/launch_round.py "$1"
