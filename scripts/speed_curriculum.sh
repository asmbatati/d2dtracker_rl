#!/usr/bin/env bash
# Speed curriculum: raise the evasive target's speed stage by stage.
#
#   scripts/speed_curriculum.sh [SPEEDS] [STEPS]
#
# Starts from checkpoints/intercept/model_evasive_40k.zip (0.5 m/s) and, for
# each speed in SPEEDS (default "1.0 2.0 3.0"), bootstraps a SAC refine on the
# evasive target at that speed (same recipe as the June curriculum: policy-
# weight transfer, fixed entropy 0.1, LR 1e-4, tight-bell reward), saves a
# checkpoint every 20k steps, evaluates each checkpoint deterministically
# (10 episodes, held-out seed 200) and carries the best one into the next stage
# as checkpoints/speed/model_evasive_<speed>.zip. Everything runs on the CPU
# and on one isolated stack (PAIR_RANK, DOMAIN_BASE, RUN_ID as in
# eval_matrix.sh). Logs: checkpoints/speed/<speed>/*.log.
set -u
SPEEDS=${1:-"1.0 2.0 3.0"}
STEPS=${2:-60000}
PAIR_RANK=${PAIR_RANK:-0}
DOMAIN_BASE=${DOMAIN_BASE:-21}
RUN_ID=${RUN_ID:-ihunter}
PKG=$(cd "$(dirname "$0")/.." && pwd)
cd "$PKG" || exit 1
INIT=${INIT:-checkpoints/intercept/model_evasive_40k.zip}
STACK=(--domain-base "$DOMAIN_BASE" --run-id "$RUN_ID")

for speed in $SPEEDS; do
  D=checkpoints/speed/$speed
  mkdir -p "$D"
  echo "$(date +%T) stage evasive@$speed from $INIT ($STEPS steps)"
  ros2 run d2dtracker_rl train_intercept --target-policy evasive --target-speed "$speed" \
    --init-from "$INIT" --env-config config/refine_tight.yaml --learning-rate 1e-4 \
    --ent-coef 0.1 --timesteps "$STEPS" --speed-factor 4 --pair-rank "$PAIR_RANK" \
    "${STACK[@]}" --device cpu --out "$D" --log-dir "$D/runs" > "$D/train.log" 2>&1
  best="" best_caps=-1
  for ck in "$D"/intercept/sac_intercept_evasive_*_steps.zip; do
    [ -f "$ck" ] || continue
    name=$(basename "$ck" .zip)
    ros2 run d2dtracker_rl evaluate_intercept --model "$ck" --target-policy evasive \
      --target-speed "$speed" --episodes 10 --seed 200 --rank "$PAIR_RANK" \
      "${STACK[@]}" --csv "$D/eval.csv" --label "$name@$speed" > "$D/eval_$name.log" 2>&1
    caps=$(grep -oP 'capture rate : \K[0-9]+' "$D/eval_$name.log" || echo 0)
    echo "    $name: $caps/10 captures"
    if [ "$caps" -gt "$best_caps" ]; then best=$ck; best_caps=$caps; fi
  done
  [ -n "$best" ] || { echo "no checkpoint for $speed"; exit 1; }
  cp "$best" "checkpoints/speed/model_evasive_$speed.zip"
  echo "$(date +%T) kept $(basename "$best") ($best_caps/10) -> checkpoints/speed/model_evasive_$speed.zip"
  INIT=checkpoints/speed/model_evasive_$speed.zip
done
