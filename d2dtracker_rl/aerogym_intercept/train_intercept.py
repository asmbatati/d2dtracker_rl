"""SB3 trainer for the interception task on the AeroGym-PX4 core.

    ros2 run d2dtracker_rl train_intercept --timesteps 200000 --speed-factor 4
    ros2 run d2dtracker_rl train_intercept --target-policy constant_velocity \
        --init-from checkpoints/intercept/final_model.zip

Curriculum axis = the target policy (static -> constant_velocity -> evasive),
chained with --init-from, exactly like the original d2dtracker_rl curriculum.
"""
import argparse
import os

import yaml


def main(args=None):
    p = argparse.ArgumentParser(description='AeroGym interception trainer')
    p.add_argument('--algo', default='sac', choices=['sac', 'ppo'])
    p.add_argument('--timesteps', type=int, default=200000)
    p.add_argument('--n-envs', type=int, default=1)
    p.add_argument('--speed-factor', type=float, default=4.0)
    p.add_argument('--target-policy', default='static',
                   choices=['static', 'constant_velocity', 'evasive'])
    p.add_argument('--env-config', default=None, help='env YAML override')
    p.add_argument('--init-from', default=None)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--out', default='checkpoints')
    p.add_argument('--log-dir', default='runs')
    args = p.parse_args(args)

    env_cfg = {}
    if args.env_config:
        with open(args.env_config) as f:
            env_cfg = yaml.safe_load(f) or {}
    env_cfg['target_policy'] = args.target_policy

    from . import task  # noqa: F401  (registers 'intercept' in aerogym TASKS)
    from .worker import make_intercept_vec_env
    env = make_intercept_vec_env(args.n_envs, env_cfg,
                                 run_id=f'icpt_{args.target_policy}',
                                 speed_factor=args.speed_factor,
                                 seed=args.seed)

    from stable_baselines3 import PPO, SAC
    from stable_baselines3.common.callbacks import CheckpointCallback

    out_dir = os.path.join(args.out, 'intercept')
    os.makedirs(out_dir, exist_ok=True)
    common = dict(verbose=1, seed=args.seed,
                  tensorboard_log=os.path.join(args.log_dir, 'intercept'))
    if args.algo == 'ppo':
        model = PPO('MlpPolicy', env, learning_rate=3e-4, n_steps=2048,
                    batch_size=64, gamma=0.99, gae_lambda=0.95, **common)
    else:
        model = SAC('MlpPolicy', env, learning_rate=3e-4,
                    buffer_size=1_000_000, batch_size=256, gamma=0.99,
                    tau=0.005, train_freq=1, **common)
    if args.init_from and os.path.isfile(args.init_from):
        model.set_parameters(args.init_from)
        print(f'[train_intercept] bootstrapped from {args.init_from}')

    ckpt = CheckpointCallback(
        save_freq=max(20000 // max(args.n_envs, 1), 1), save_path=out_dir,
        name_prefix=f'{args.algo}_intercept_{args.target_policy}')
    print(f'[train_intercept] algo={args.algo} target={args.target_policy} '
          f'timesteps={args.timesteps} n_envs={args.n_envs} -> {out_dir}')
    try:
        model.learn(total_timesteps=args.timesteps, callback=ckpt)
    finally:
        final = os.path.join(out_dir, 'final_model.zip')
        model.save(final)
        print(f'[train_intercept] saved {final}')
        env.close()


if __name__ == '__main__':
    main()
