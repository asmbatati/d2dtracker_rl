# Interception RL results

Drone-to-drone interception trained on the AeroGym-PX4 sim (PX4 SITL in the
loop, two vehicles per worker, sim-time stepping at 4x). SAC, velocity action
interface, managed two-vehicle stacks.

## Pipeline validation

The full RL loop works end-to-end: the `InterceptSimInstance` boots an
interceptor + a scripted target in one gz world, `InterceptEnv` exposes the
relative-state observation, and SB3 trains a capture policy with the AeroGym
reliability machinery (zero wedges across all runs).

Pre-training sanity: a hand-coded P-controller chase captured a static target
in **6.3 s** sim — confirming the action/observation/reward wiring.

## Eval-reset robustness fix

The single-env evaluator used to produce spurious 0.06–0.17 s episodes after the
first terminal episode: a crash/out-of-bounds left the craft grounded (arming
lockout) or escaped (airborne, far from spawn), and the old reset only checked
"airborne", so it started the next episode wherever the craft was → instant
re-termination, cascading. Fixed in `aerogym_px4` `BasePX4Env._ensure_ready`:
every reset now verifies the craft is **finite + in-bounds + near spawn + armed**
and escalates per-episode — airborne→teleport-home, grounded→level-and-takeoff,
and a **stack restart** fallback for the escaped/EKF-diverged case. Result: every
eval episode is a full-length attempt again (6/6 clean, consistent min-ranges).

## Reward rework

The first reward (closing + range + alignment + a narrow precision bell) trained
a policy that looked fine in stochastic training (ep_len 400, no crashes) but
**sank into the ground when evaluated deterministically** — SAC's exploration
noise had been holding it aloft. Reworked in `aerogym_intercept/task.py`:

* **dense altitude-floor penalty** above the crash line — a continuous upward
  gradient so the deterministic policy never learns to sink (the one-shot −1000
  crash was too sparse for the value function under exploration);
* **dense arena-boundary penalty** near the walls/ceiling — same idea, stops the
  policy overshooting the target straight out of bounds;
* **wide, strong near-field bell** (`prec_sigma` 4.0, `w_prec` 6.0) + a small
  constant inward `w_range` pull — a narrow bell parked the policy at ~3.8 m
  (outside the bell, where its gradient is flat); the wide bell pulls the
  equilibrium all the way into the capture gate.

Each change was driven by an observed eval failure (sink → floor; overshoot-oob
→ boundary; park-at-3.8 m → wide bell), mirroring how the AeroGym hover task was
shaped over several iterations.

## Stage 1 — static target (SAC), reward-shaping trajectory

All evals are 6 deterministic episodes, capture gate 1.0 m.

| policy / budget | capture | mean min-range | behaviour |
|---|---|---|---|
| v1 (old reward, 450k) | 2/6 | — | deterministic **sink & crash** |
| v3 80k (floor + boundary) | 0/6 | 3.78 m | stable, in-bounds, parks too far |
| v4 60k (wide strong bell) | 1/6 | 57 m | captures but **overshoots/oob** (under-trained, aggressive) |
| v4 100k | 2/6 | 1.17 m | stable, tight, all 6 within 1.5 m |
| **v4 120k (best)** | **2/6** | **1.05 m** | stable, all 6 within 1.36 m, 0 crash / 0 oob |

The v4 policy reliably closes to the **1.0 m capture boundary** and dips under it
on 2/6 (one near-miss was exactly 1.00 m). 0 crashes, 0 out-of-bounds, 6/6 clean
resets — a control-precision limit at the equilibrium, not instability.

### SAC stability note (use early stopping)

Past the ~120 k peak (ep_rew_mean +682) the run **diverged** — ep_rew collapsed
to −1790 by 140 k (Q-overestimation from the large +1000 capture target combined
with auto-entropy, a known SAC failure mode). The per-20k checkpoints preserve
the pre-collapse models; **`model_static_v4_best.zip` (120 k) is the deliverable.**
To push capture rate higher / train longer safely: lower the learning rate
(3e-4 → 1e-4), shrink the capture bonus, or bootstrap from the 120 k model with a
tighter near-field bell to nudge the equilibrium under 1.0 m.

## Curriculum (planned)

Target-policy axis, each stage bootstrapped from the previous checkpoint:
`static` -> `constant_velocity` -> `evasive` (the `target_policy.py`
behaviours). Run with `--target-policy ... --init-from ...`. Bootstrap transfers
network weights only with a fixed low entropy coef (see `train_intercept.py`).

## Environment note

MAVROS must be rebuilt after a system `diagnostic_updater` update (ABI change) or
every boot fails with `boot failed: ['mavros']` (undefined-symbol, exit 127):
`colcon build --packages-select mavros mavros_extras`.

## Reproduce

```bash
# train (stop at / keep the best ~100-120k checkpoint; later checkpoints collapse)
ros2 run d2dtracker_rl train_intercept --timesteps 150000 --speed-factor 4 --pair-rank 0
# evaluate deterministically on a disjoint stack
ros2 run d2dtracker_rl evaluate_intercept \
    --model checkpoints/intercept/model_static_v4_best.zip \
    --episodes 6 --target-policy static --rank 2
```
