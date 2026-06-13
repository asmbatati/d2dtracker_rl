# Interception RL results

Drone-to-drone interception trained on the AeroGym-PX4 sim (PX4 SITL in the
loop, two vehicles per worker, sim-time stepping at 4x). SAC, velocity action
interface, managed two-vehicle stacks.

## Pipeline validation

The full RL loop works end-to-end: the `InterceptSimInstance` boots an
interceptor + a scripted target in one gz world, `InterceptEnv` exposes the
relative-state observation, and SB3 trains a capture policy with the AeroGym
reliability machinery (zero wedges/crashes across all runs).

Pre-training sanity: a hand-coded P-controller chase captured a static target
in **6.3 s** sim — confirming the action/observation/reward wiring.

## Stage 1 — static target (SAC)

| budget | eval (6 deterministic eps) | notes |
|---|---|---|
| 200k | **2/6 capture**, mean closest approach 2.65 m (best 0.87 m), mean t_capture 48 s | reward still climbing (-1020 -> -187); under-trained |
| +250k (running) | — | continuation from the 200k checkpoint |

Reading: the policy reliably *approaches* (closest approach 0.9-1.8 m on near
intercepts) but the final-meter precision is the limiter — the same near-field
gradient flattening seen in the hover task. Levers: more training (reward had
not plateaued), tighten the precision bell (`prec_sigma` 3.0 -> ~1.5,
`w_prec`), or a terminal-approach speed-damping term.

## Curriculum (planned)

Target-policy axis, each stage bootstrapped from the previous checkpoint:
`static` -> `constant_velocity` -> `evasive` (the `target_policy.py`
behaviours). Run with `--target-policy ... --init-from ...`.

## Reproduce

```bash
ros2 run d2dtracker_rl train_intercept --timesteps 200000 --speed-factor 4 --pair-rank 1
ros2 run d2dtracker_rl evaluate_intercept --model checkpoints/intercept/final_model.zip \
    --episodes 10 --target-policy static --rank 1
```
