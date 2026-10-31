#!/usr/bin/env bash
# Staged SAC curriculum with checkpoint selection, on one isolated stack.
#
#   scripts/curriculum.sh OUT_DIR "STAGES" [STEPS]
#
# STAGES is a list of target_policy:speed. Each stage bootstraps a SAC refine
# from the previous stage's model (the first from INIT) with the June recipe
# (policy-weight transfer, fixed entropy 0.1, LR 1e-4, tight-bell reward),
# saves a checkpoint every 20k steps, evaluates each checkpoint
# deterministically (EVAL_EPS episodes on the selection seed SELECT_SEED, kept
# apart from eval_matrix.sh's test seed 200) and keeps the best one as
# OUT_DIR/model_<target>_<speed>.zip for the next stage.
#
#   # C9: the June refine stages again, from the June static model, with an
#   # exact 0.05 s control period (lockstep)
#   scripts/curriculum.sh checkpoints/lockstep "static:0 constant_velocity:0.5 evasive:0.5" 40000
#   # C6: speed curriculum, continuing from the evasive model
#   INIT=checkpoints/lockstep/model_evasive_0.5.zip \
#     scripts/curriculum.sh checkpoints/speed "evasive:1.0 evasive:2.0 evasive:3.0" 60000
#   # C11: C9 with 55 s episodes (1100 steps), the length the June models had
#   EPISODE_STEPS=1100 \
#     scripts/curriculum.sh checkpoints/lockstep_55s "static:0 constant_velocity:0.5 evasive:0.5" 40000
#
# Everything runs on the CPU (the GPU stays free for other sessions)
# and on one stack: PAIR_RANK, DOMAIN_BASE, RUN_ID as in eval_matrix.sh.
# STEP_MODE defaults to lockstep; EPISODE_STEPS (env default 400) sets the
# episode length for training and for checkpoint selection. Logs: OUT_DIR/<target>_<speed>/*.log.
set -u
OUT=${1:?usage: curriculum.sh OUT_DIR "STAGES" [STEPS]}
STAGES=${2:?usage: curriculum.sh OUT_DIR "STAGES" [STEPS]}
STEPS=${3:-40000}
PAIR_RANK=${PAIR_RANK:-0}
DOMAIN_BASE=${DOMAIN_BASE:-21}
RUN_ID=${RUN_ID:-ihunter}
STEP_MODE=${STEP_MODE:-lockstep}
SPEED_FACTOR=${SPEED_FACTOR:-4}
EVAL_EPS=${EVAL_EPS:-10}
SELECT_SEED=${SELECT_SEED:-100}
EPISODE_STEPS=${EPISODE_STEPS:-}
PKG=$(cd "$(dirname "$0")/.." && pwd)
cd "$PKG" || exit 1
INIT=${INIT:-checkpoints/intercept/model_static_refine_40k.zip}
STACK=(--domain-base "$DOMAIN_BASE" --run-id "$RUN_ID" --speed-factor "$SPEED_FACTOR"
       --step-mode "$STEP_MODE" ${EPISODE_STEPS:+--episode-steps "$EPISODE_STEPS"})

for stage in $STAGES; do
  target=${stage%%:*} speed=${stage##*:}
  D=$OUT/${target}_$speed
  keep=$OUT/model_${target}_$speed.zip
  mkdir -p "$D"
  if [ -f "$keep" ]; then   # resume: stage already done
    echo "skip $stage ($keep exists)"; INIT=$keep; continue
  fi
  echo "$(date +%T) stage $target@$speed from $INIT ($STEPS steps, $STEP_MODE)"
  ros2 run d2dtracker_rl train_intercept --target-policy "$target" --target-speed "$speed" \
    --init-from "$INIT" --env-config config/refine_tight.yaml --learning-rate 1e-4 \
    --ent-coef 0.1 --timesteps "$STEPS" --pair-rank "$PAIR_RANK" "${STACK[@]}" \
    --device cpu --out "$D" --log-dir "$D/runs" > "$D/train.log" 2>&1
  best="" best_caps=-1
  for ck in "$D"/intercept/sac_intercept_"${target}"_*_steps.zip; do
    [ -f "$ck" ] || continue
    name=$(basename "$ck" .zip)
    ros2 run d2dtracker_rl evaluate_intercept --model "$ck" --target-policy "$target" \
      --target-speed "$speed" --episodes "$EVAL_EPS" --seed "$SELECT_SEED" \
      --rank "$PAIR_RANK" "${STACK[@]}" --csv "$D/select.csv" --label "$name@$speed" \
      > "$D/select_$name.log" 2>&1
    caps=$(grep -oP 'capture rate : \K[0-9]+' "$D/select_$name.log" || echo 0)
    echo "    $name: $caps/$EVAL_EPS captures"
    if [ "$caps" -gt "$best_caps" ]; then best=$ck; best_caps=$caps; fi
  done
  [ -n "$best" ] || { echo "no checkpoint for $stage"; exit 1; }
  cp "$best" "$keep"
  echo "$(date +%T) kept $(basename "$best") ($best_caps/$EVAL_EPS) -> $keep"
  INIT=$keep
done
