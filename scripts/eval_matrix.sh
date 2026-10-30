#!/usr/bin/env bash
# Evaluate the curriculum policies and the scripted baselines on the same seeds.
#
#   scripts/eval_matrix.sh OUT_DIR [EPISODES] [SPEEDS]
#
# For every target policy (static, constant_velocity, evasive) and target speed
# in SPEEDS (moving targets only; default "0.5"), runs evaluate_intercept for
# the three curriculum models plus --policy pn, pn_int and pursuit (and, with
# CBF=1, the default, PN / pursuit / the evasive model behind the CBF safety
# filter, labelled <name>+cbf), appending
# one row per episode to OUT_DIR/episodes.csv and saving trajectories under
# OUT_DIR/traj/. Cells that already have EPISODES rows are skipped, so the
# script can be re-run to resume.
#
# Isolation (shared machine): PAIR_RANK selects PX4 instances 2r/2r+1,
# DOMAIN_BASE + 2*PAIR_RANK is the stack's ROS_DOMAIN_ID, RUN_ID names the gz
# partition (aerogym_<RUN_ID>_<2r>). Defaults: rank 1 (instances 2,3),
# domain 21, run id ihunter_eval.
set -u
OUT=${1:?usage: eval_matrix.sh OUT_DIR [EPISODES] [SPEEDS]}
EPS=${2:-30}
SPEEDS=${3:-0.5}
PAIR_RANK=${PAIR_RANK:-1}
DOMAIN_BASE=${DOMAIN_BASE:-19}
RUN_ID=${RUN_ID:-ihunter_eval}
SEED=${SEED:-200}
CBF=${CBF:-1}
PKG=$(cd "$(dirname "$0")/.." && pwd)
CK=${CKPT_DIR:-$PKG/checkpoints/intercept}
mkdir -p "$OUT/traj" "$OUT/logs"
CSV=$OUT/episodes.csv

rows() {  # rows for (label, target, speed) already in the CSV
  [ -f "$CSV" ] || { echo 0; return; }
  awk -F, -v p="$1" -v t="$2" -v s="$3" \
    'NR>1 && $1==p && $3==t && ($4+0)==(s+0) {n++} END {print n+0}' "$CSV"
}

run_cell() {  # label target speed args...
  local label=$1 target=$2 speed=$3; shift 3
  local have
  have=$(rows "$label" "$target" "$speed")
  if [ "$have" -ge "$EPS" ]; then
    echo "skip $label $target $speed ($have rows)"; return
  fi
  echo "$(date +%T) $label target=$target speed=$speed eps=$EPS"
  ros2 run d2dtracker_rl evaluate_intercept "$@" --label "$label" \
    --target-policy "$target" --target-speed "$speed" --episodes "$EPS" \
    --seed "$SEED" --rank "$PAIR_RANK" --domain-base "$DOMAIN_BASE" \
    --run-id "$RUN_ID" --csv "$CSV" --traj-dir "$OUT/traj" \
    > "$OUT/logs/${label}__${target}_${speed}.log" 2>&1
  tail -4 "$OUT/logs/${label}__${target}_${speed}.log" | sed 's/^/    /'
}

for target in static constant_velocity evasive; do
  if [ "$target" = static ]; then speeds=0; else speeds=$SPEEDS; fi
  for speed in $speeds; do
    for m in model_static_refine_40k model_cv_40k model_evasive_40k; do
      run_cell "$m" "$target" "$speed" --model "$CK/$m.zip"
    done
    run_cell pn "$target" "$speed" --policy pn
    run_cell pn_int "$target" "$speed" --policy pn_int
    run_cell pursuit "$target" "$speed" --policy pursuit
    if [ "$CBF" = 1 ]; then
      run_cell pn+cbf "$target" "$speed" --policy pn --cbf
      run_cell pursuit+cbf "$target" "$speed" --policy pursuit --cbf
      run_cell model_evasive_40k+cbf "$target" "$speed" --model "$CK/model_evasive_40k.zip" --cbf
    fi
  done
done
echo "$(date +%T) done -> $CSV"
