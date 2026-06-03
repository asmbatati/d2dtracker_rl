#!/usr/bin/env python3
"""Run the interception RL curriculum: a sequence of training stages.

Reads a curriculum YAML (see config/curriculum.yaml) and runs each stage as a
fresh `train` process (one process per stage keeps PX4/rclpy state clean),
chaining checkpoints so later stages bootstrap from earlier ones.

The simulation must already be running (one drone for high-level, two for the
interception scenario). Example:

  ros2 run d2dtracker_rl curriculum --config <pkg>/config/curriculum.yaml
"""
import argparse
import os
import subprocess
import sys

import yaml


def main(argv=None):
    parser = argparse.ArgumentParser(description='Run the RL interception curriculum.')
    parser.add_argument('--config', required=True, help='curriculum YAML path')
    parser.add_argument('--ckptdir', default='checkpoints')
    parser.add_argument('--logdir', default='runs')
    parser.add_argument('--timesteps', type=int, default=0,
                        help='override per-stage total_timesteps (0 = use config)')
    parser.add_argument('--start', type=int, default=0,
                        help='index of the first stage to run (resume)')
    args, _ = parser.parse_known_args(argv)

    with open(args.config) as f:
        spec = yaml.safe_load(f) or {}
    stages = spec.get('stages', [])
    if not stages:
        print('No stages in curriculum config.', file=sys.stderr)
        return 1

    cfg_dir = os.path.dirname(os.path.abspath(args.config))
    prev_ckpt = ''

    for i, stage in enumerate(stages):
        if i < args.start:
            # Still advance the bootstrap pointer so --start resumes correctly.
            name = stage.get('name', f'stage{i}')
            prev_ckpt = os.path.join(args.ckptdir, name, 'final_model.zip')
            continue
        name = stage.get('name', f'stage{i}')
        env_stage = stage.get('env_stage', 'high')
        stage_cfg = stage.get('config', '')
        if stage_cfg and not os.path.isabs(stage_cfg):
            stage_cfg = os.path.join(cfg_dir, stage_cfg)
        init_from = stage.get('init_from', '') or prev_ckpt

        cmd = ['ros2', 'run', 'd2dtracker_rl', 'train',
               '--stage', env_stage, '--name', name,
               '--ckptdir', args.ckptdir, '--logdir', args.logdir]
        if stage_cfg:
            cmd += ['--config', stage_cfg]
        if stage.get('target_policy'):
            cmd += ['--target-policy', stage['target_policy']]
        if args.timesteps:
            cmd += ['--timesteps', str(args.timesteps)]
        if init_from:
            cmd += ['--init-from', init_from]

        print(f'\n=== Curriculum stage {i}: {name} ===\n{" ".join(cmd)}\n', flush=True)
        result = subprocess.run(cmd)
        if result.returncode != 0:
            print(f'Stage {name} failed (rc={result.returncode}); stopping.',
                  file=sys.stderr)
            return result.returncode
        prev_ckpt = os.path.join(args.ckptdir, name, 'final_model.zip')

    print('\nCurriculum complete.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
