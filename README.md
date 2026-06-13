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
| `aerogym_intercept/train_intercept.py` / `evaluate_intercept.py` | SB3 trainer (curriculum over target policies) + capture-rate evaluator |
| `target_policy.py` | scripted target: `static` / `constant_velocity` / `evasive` (the curriculum axis) |

`external/AeroGym-PX4` is a git **submodule** pinning the exact playground
version (it carries a `COLCON_IGNORE`; the colcon workspace builds the
side-by-side `src/AeroGym-PX4` checkout — keep exactly one buildable copy).

## Train / evaluate

```bash
# stage 1: static target (SAC, managed two-vehicle stack, 4x sim)
ros2 run d2dtracker_rl train_intercept --timesteps 200000 --speed-factor 4
# stage 2/3: moving / evasive target, bootstrapped:
ros2 run d2dtracker_rl train_intercept --target-policy constant_velocity \
    --init-from checkpoints/intercept/final_model.zip
ros2 run d2dtracker_rl evaluate_intercept --model checkpoints/intercept/final_model.zip \
    --episodes 10 --target-policy constant_velocity
```

Visualize live training (Gazebo GUI spectator), TensorBoard, capacity and
troubleshooting: see AeroGym-PX4's `TRAINING.md` (partitions here are
`aerogym_icpt_<policy>_<2r>`). Each worker pair costs ~5-6 CPU cores.

## Legacy

`d2dtracker_rl/envs/` (the pre-AeroGym single-process envs), `train.py`,
`evaluate.py`, `curriculum.py` and the launch files are the original
implementation, kept for reference; new work targets `aerogym_intercept/`.
