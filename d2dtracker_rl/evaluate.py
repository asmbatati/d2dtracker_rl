#!/usr/bin/env python3
"""Evaluate a trained interception policy against the running (rendered) sim.

  ros2 run d2dtracker_rl evaluate --stage high --model checkpoints/high/final_model.zip
"""
import argparse
import os

import numpy as np
import rclpy
import yaml

from stable_baselines3 import PPO, SAC

from .envs.high_level_env import HighLevelInterceptEnv
from .envs.low_level_env import LowLevelInterceptEnv

ALGOS = {'PPO': PPO, 'SAC': SAC}
ENVS = {'high': HighLevelInterceptEnv, 'low': LowLevelInterceptEnv}


def main(argv=None):
    parser = argparse.ArgumentParser(description='Evaluate an interception policy.')
    parser.add_argument('--stage', choices=list(ENVS), default='high')
    parser.add_argument('--model', required=True)
    parser.add_argument('--config', default='')
    parser.add_argument('--episodes', type=int, default=10)
    parser.add_argument('--algo', default='')
    args, _ = parser.parse_known_args(argv)

    env_cfg = {}
    if args.config and os.path.exists(args.config):
        with open(args.config) as f:
            env_cfg = (yaml.safe_load(f) or {}).get('env', {})

    rclpy.init()
    env = ENVS[args.stage](config=env_cfg)
    algo_name = args.algo or ('PPO' if args.stage == 'high' else 'SAC')
    model = ALGOS[algo_name].load(args.model)

    captures, final_ranges = 0, []
    try:
        for ep in range(args.episodes):
            obs, _ = env.reset()
            done = False
            info = {}
            while not done:
                action, _ = model.predict(obs, deterministic=True)
                obs, _, terminated, truncated, info = env.step(action)
                done = terminated or truncated
            outcome = info.get('outcome', 'unknown')
            final_ranges.append(info.get('range', float('nan')))
            captures += int(outcome == 'capture')
            print(f'episode {ep}: outcome={outcome} final_range={info.get("range"):.2f}')
    finally:
        rate = captures / max(args.episodes, 1)
        print(f'capture_rate={rate:.2%} mean_final_range={np.nanmean(final_ranges):.2f}')
        env.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
