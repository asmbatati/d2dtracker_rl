#!/usr/bin/env python3
"""Stable-Baselines3 training driver for drone-to-drone interception.

Assumes the simulation (interceptor + target in one Gazebo world, both MAVROS
stacks up) is ALREADY running. Brings up a single PX4-in-the-loop Gymnasium env
(n_envs=1; PX4 SITL cannot be massively parallelized) and trains PPO (high-level
stage) or SAC (low-level stage).

  ros2 run d2dtracker_rl train --stage high --config <pkg>/config/train_ppo_high_level.yaml
  ros2 run d2dtracker_rl train --stage low  --config <...>/train_sac_low_level.yaml \
        --init-from checkpoints/high/best_model.zip
"""
import argparse
import os

import rclpy
import yaml

from stable_baselines3 import PPO, SAC
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor

from .envs.high_level_env import HighLevelInterceptEnv
from .envs.low_level_env import LowLevelInterceptEnv

ALGOS = {'PPO': PPO, 'SAC': SAC}
ENVS = {'high': HighLevelInterceptEnv, 'low': LowLevelInterceptEnv}


def load_config(path):
    if path and os.path.exists(path):
        with open(path) as f:
            return yaml.safe_load(f) or {}
    return {}


def main(argv=None):
    parser = argparse.ArgumentParser(description='Train an interception policy.')
    parser.add_argument('--stage', choices=list(ENVS), default='high')
    parser.add_argument('--config', default='')
    parser.add_argument('--timesteps', type=int, default=0,
                        help='override total_timesteps from config')
    parser.add_argument('--logdir', default='runs')
    parser.add_argument('--ckptdir', default='checkpoints')
    parser.add_argument('--init-from', default='',
                        help='load weights from a prior model (curriculum bootstrap)')
    parser.add_argument('--target-policy', default='',
                        help='override env target_policy (static|constant_velocity|evasive)')
    parser.add_argument('--name', default='',
                        help='run name -> runs/<name> and checkpoints/<name>')
    args, _ = parser.parse_known_args(argv)

    cfg = load_config(args.config)
    env_cfg = cfg.get('env', {})
    algo_cfg = cfg.get('algo', {})
    algo_name = algo_cfg.pop('name', 'PPO' if args.stage == 'high' else 'SAC')
    total_timesteps = args.timesteps or algo_cfg.pop('total_timesteps', 100_000)
    if args.target_policy:
        env_cfg['target_policy'] = args.target_policy

    run_name = args.name or args.stage

    rclpy.init()
    env = Monitor(ENVS[args.stage](config=env_cfg))

    algo_cls = ALGOS[algo_name]
    tb = os.path.join(args.logdir, run_name)
    if args.init_from and os.path.exists(args.init_from):
        model = algo_cls.load(args.init_from, env=env, tensorboard_log=tb)
        print(f'Loaded initial weights from {args.init_from}')
    else:
        model = algo_cls('MlpPolicy', env, verbose=1, tensorboard_log=tb, **algo_cfg)

    ckpt = CheckpointCallback(
        save_freq=max(total_timesteps // 20, 1000),
        save_path=os.path.join(args.ckptdir, run_name),
        name_prefix='model')

    try:
        model.learn(total_timesteps=total_timesteps, callback=ckpt,
                    progress_bar=False)
    except KeyboardInterrupt:
        print('Interrupted; saving current model.')
    finally:
        out = os.path.join(args.ckptdir, run_name, 'final_model')
        model.save(out)
        print(f'Saved model to {out}.zip')
        env.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
