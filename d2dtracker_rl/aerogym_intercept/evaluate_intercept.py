"""Evaluate an interception policy: capture rate / time / min range.

    ros2 run d2dtracker_rl evaluate_intercept \
        --model checkpoints/intercept/final_model.zip --episodes 10 \
        --target-policy constant_velocity
"""
import argparse

import numpy as np


def main(args=None):
    p = argparse.ArgumentParser(description='AeroGym interception evaluator')
    p.add_argument('--model', required=True)
    p.add_argument('--episodes', type=int, default=10)
    p.add_argument('--target-policy', default='static')
    p.add_argument('--speed-factor', type=float, default=4.0)
    p.add_argument('--rank', type=int, default=2,
                   help='pair rank for the eval stack (avoid training ranks)')
    p.add_argument('--seed', type=int, default=100)
    args = p.parse_args(args)

    from . import task  # noqa: F401
    from .worker import build_intercept_worker_env
    env = build_intercept_worker_env(
        args.rank, {'target_policy': args.target_policy},
        run_id='icpt_eval', speed_factor=args.speed_factor, stagger=0.0)

    from stable_baselines3 import PPO, SAC
    model = None
    for cls in (SAC, PPO):
        try:
            model = cls.load(args.model)
            break
        except Exception:
            continue
    if model is None:
        raise SystemExit(f'could not load {args.model}')

    captures, times, min_ranges = 0, [], []
    for ep in range(args.episodes):
        obs, _ = env.reset(seed=args.seed + ep)
        done = trunc = False
        min_rng = float('inf')
        while not (done or trunc):
            action, _ = model.predict(obs, deterministic=True)
            obs, r, done, trunc, info = env.step(action)
            st = env._state()
            min_rng = min(min_rng, float(np.linalg.norm(
                st['target_position'] - st['position'])))
        outcome = info.get('outcome', 'timeout')
        if outcome == 'capture':
            captures += 1
            times.append(env._t)
        min_ranges.append(min_rng)
        print(f'  ep {ep:2d}: outcome={outcome:13s} t={env._t:6.2f}s '
              f'min_range={min_rng:6.2f} m')

    print(f'\n==== intercept eval ({args.episodes} eps, '
          f'target={args.target_policy}) ====')
    print(f'  capture rate : {captures}/{args.episodes}')
    if times:
        print(f'  mean t_capture: {np.mean(times):.2f} s')
    print(f'  mean min_range: {np.mean(min_ranges):.2f} m')
    env.close()


if __name__ == '__main__':
    main()
