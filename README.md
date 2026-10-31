# d2dtracker_rl

Reinforcement learning for **drone-to-drone interception** (Gymnasium + SB3),
built on the [AeroGym-PX4](https://github.com/asmbatati/AeroGym-PX4)
autopilot-in-the-loop playground (PX4 SITL + ROS 2 Jazzy + MAVROS + gz
Harmonic).

## Architecture

AeroGym-PX4 provides the generic single-vehicle training core (sim-time
stepping at a capped speed factor, managed isolated sim stacks with crash
auto-restart, reset/rescue machinery, SB3 trainers). This package adds the
interception layer on top — the playground itself stays interception-free:

| piece | what |
|---|---|
| `aerogym_intercept/sim.py` | `InterceptSimInstance` — TWO PX4 vehicles in one gz world per worker (instances `2r`/`2r+1`; pair ranks 0..4) |
| `aerogym_intercept/env.py` | `InterceptEnv(BasePX4Env)` — adds the target's ROS interface, world-frame target state, scripted target driving, target episode reset |
| `aerogym_intercept/task.py` | `intercept` task plugin (registered into aerogym's registry from here): closing + alignment + near-field precision reward, capture termination |
| `aerogym_intercept/train_intercept.py` | SB3 trainer (curriculum over target policies; `--init-from` bootstrap, `--device`, `--domain-base`/`--run-id` for stack isolation) |
| `aerogym_intercept/evaluate_intercept.py` | deterministic evaluator: `--policy model` or the scripted baselines `pn` / `pursuit` in the same env and seeds; `--cbf` safety filter; `--csv` per-episode rows, `--traj-dir` trajectories |
| `aerogym_intercept/policy_head.py` | flies a trained policy in `drone_interception_sim`'s two-drone benchmark scene (`pipeline.launch.py orchestrator:=rl rl_model:=...`) |
| `target_policy.py` | scripted target: `static` / `constant_velocity` / `evasive` (the curriculum axis) |
| `scripts/eval_matrix.sh` | all curriculum models + PN + pursuit over targets and speeds, resumable CSV |
| `scripts/curriculum.sh` | staged SAC curriculum (`target:speed` stages, e.g. `static:0 constant_velocity:0.5 evasive:0.5` or evasive speed stages), lockstep by default; keeps the best checkpoint per stage, selected on seed 100 (the evaluation matrix tests on seed 200) |
| `scripts/plot_training.py` | learning curves from the SB3 train logs → `docs/figures/learning_curves.png` |
| `tools/video_montage/` | record → replay (offscreen GPU camera) → captioned montage of the policy across checkpoints |

`external/AeroGym-PX4` is a git **submodule** pinning the exact playground
version (it carries a `COLCON_IGNORE`; the colcon workspace builds the
side-by-side `src/AeroGym-PX4` checkout — keep exactly one buildable copy).

## Train / evaluate

Run from this package's directory (checkpoint paths are relative). Each stack
is self-managed: pair rank r uses PX4 instances 2r/2r+1, ROS domain
`--domain-base + 2r` (default 100) and gz partition `aerogym_<run-id>_<2r>`;
on a shared machine pick values nobody else uses. `RMW_IMPLEMENTATION` is
forced to FastRTPS inside the stack.

```bash
# stage 1: static target (SAC, managed two-vehicle stack, 4x sim)
ros2 run d2dtracker_rl train_intercept --timesteps 150000 --speed-factor 4 --pair-rank 0
# then refine / move on, bootstrapping the best checkpoint (see RESULTS.md for the
# exact June 2026 recipe: --env-config config/refine_tight.yaml --learning-rate 1e-4
# --ent-coef 0.1), e.g.:
ros2 run d2dtracker_rl train_intercept --target-policy constant_velocity \
    --init-from checkpoints/intercept/model_static_refine_40k.zip \
    --env-config config/refine_tight.yaml --learning-rate 1e-4 --ent-coef 0.1 \
    --timesteps 100000 --speed-factor 4 --pair-rank 0 --device cpu
# deterministic evaluation (learned policy, then the baselines on the same seeds)
ros2 run d2dtracker_rl evaluate_intercept --model checkpoints/intercept/model_evasive_40k.zip \
    --target-policy evasive --target-speed 0.5 --episodes 30 --seed 200 --rank 1 --csv eval.csv
ros2 run d2dtracker_rl evaluate_intercept --policy pn --target-policy evasive --episodes 30 \
    --seed 200 --rank 1 --csv eval.csv
```

Results: [`RESULTS.md`](RESULTS.md). The three curriculum deliverables are in
git (`checkpoints/intercept/model_{static_refine,cv,evasive}_40k.zip`); every
other checkpoint, log and TensorBoard run stays on disk (gitignored).

**Velocity frame note.** AeroGym builds the observation from MAVROS
`local_position/odom`, whose twist is in the BODY frame. The interceptor
velocity in the observation is therefore body-frame, identically in training,
evaluation and `policy_head`, so the trained policies are consistent. The
target velocity is rotated to the world frame (the training target holds yaw
0, so this was a no-op for training but matters for a yawing target).

Visualize live training (Gazebo GUI spectator), TensorBoard, capacity and
troubleshooting: see AeroGym-PX4's `TRAINING.md` (partitions here are
`aerogym_icpt_<policy>_<2r>`). Each worker pair costs ~5-6 CPU cores.

## Legacy

`d2dtracker_rl/envs/` (the pre-AeroGym single-process envs), `train.py`,
`evaluate.py`, `curriculum.py` and the launch files are the original
implementation, kept for reference; new work targets `aerogym_intercept/`.
