# d2dtracker_rl

Reinforcement-learning **drone-to-drone interception** for ROS 2 Jazzy +
Gazebo Harmonic + PX4 SITL, using **Gymnasium** + **Stable-Baselines3**.

The interceptor learns to catch a target UAV with PX4 **in the loop** and the
sim **rendered** (non-headless). A **curriculum** trains two action interfaces:

| Stage | Env id | Action | Obs | Algo |
|---|---|---|---|---|
| 1 (primary) | `D2DIntercept-HighLevel-v0` | ENU velocity setpoint `Box(3)` | 13 | PPO |
| 2 | `D2DIntercept-LowLevel-v0` | body rates + thrust `Box(4)` | 20 | SAC |

## Architecture

- `ros_interface.py` — one rclpy node spun in a background thread; caches
  interceptor/target odometry + FCU state, publishes setpoints, arms / sets mode.
- `envs/base_intercept_env.py` — Gymnasium env; `step()` applies the action,
  lets the sim advance one control tick, then reads fresh cached state for the
  observation/reward (the fix over the old code, which never refreshed state).
- `envs/high_level_env.py`, `envs/low_level_env.py` — the two action interfaces.
- `px4_reset.py` — episode reset by teleporting both Gazebo entities
  (`gz service .../set_pose`) + re-establishing armed/OFFBOARD, with a
  land+rearm fallback.
- `target_policy.py` — scripted target motion (static / constant-velocity /
  evasive) for the curriculum.
- `train.py`, `evaluate.py` — SB3 drivers.

## Dependencies (pip — no apt package on Jazzy)

```bash
pip install --user --break-system-packages "stable-baselines3>=2.3" "gymnasium>=0.29" tensorboard
# torch is required by SB3; if you hit a sympy `equal_valued` ImportError, run:
pip install --user --break-system-packages --force-reinstall --no-deps "sympy>=1.13"
```

## Run (two terminals — keep the sim up across training restarts)

```bash
# Terminal A: bring up the rendered two-drone sim and wait for both FCUs
ros2 launch drone_interception_sim interception.launch.py
#   verify: ros2 topic echo /interceptor/mavros/state --once   (connected: true)

# Terminal B: stage-1 training
ros2 launch d2dtracker_rl train_high_level.launch.py
tensorboard --logdir runs/
```

Evaluate / visualise a trained policy live in Gazebo:

```bash
ros2 launch d2dtracker_rl evaluate.launch.py \
    stage:=high model:=checkpoints/high/final_model.zip
```

## Curriculum

`config/curriculum.yaml` lists progressively harder target behaviour
(static → constant-velocity → evasive). Advance stages by editing
`env.target_policy` in the train config, and bootstrap a stage from a prior
checkpoint with `--init-from checkpoints/high/final_model.zip`. Stage 2
(low-level) differs in obs/action dims, so it bootstraps behaviourally rather
than by direct weight transfer (see plan notes).
