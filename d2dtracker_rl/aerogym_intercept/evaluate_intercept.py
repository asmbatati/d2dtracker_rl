"""Evaluate an interception policy: capture rate / time / min range.

    ros2 run d2dtracker_rl evaluate_intercept \
        --model checkpoints/intercept/final_model.zip --episodes 10 \
        --target-policy constant_velocity

Scripted baselines run in the SAME env, on the same seeds and action interface
(normalized ENU velocity x ``v_max``), so their rows compare directly with the
learned policy's:

* ``--policy pn``      -- the PN velocity law of ``d2dtracker_interception``
  (N 3, closing speed 6 m/s, cap ``v_max``), the sim benchmark's PN baseline;
* ``--policy pursuit`` -- pure pursuit: fly the line of sight at ``v_max``.

``--cbf`` passes every action (learned or scripted) through the CBF safety
filter of ``d2dtracker_interception.safety`` (altitude floor ``--cbf-z-min``,
ceiling and geofence just inside the env's arena bounds).

``--csv`` appends one row per episode; ``--traj-dir`` saves each episode's
positions (``t, interceptor xyz, target xyz``) as ``.npz`` for figures.
"""
import argparse
import csv
import os

import numpy as np

CSV_FIELDS = ['policy', 'model', 'target_policy', 'target_speed', 'episode',
              'seed', 'outcome', 't_s', 'min_range_m', 'steps', 'min_alt_m',
              'cbf_active_frac']


class ScriptedPolicy:
    """Velocity-space baseline producing the env's normalized action."""

    def __init__(self, kind, env, nav_gain=3.0, closing_speed=6.0):
        self.kind = kind
        self.env = env
        self.v_max = float(env.cfg.get('v_max', 8.0))
        self.dt = float(env.control_dt)
        self.nav_gain = nav_gain
        self.closing_speed = closing_speed

    def predict(self, obs, deterministic=True):
        st = self.env._state()
        ipos, ivel = st['position'], st['velocity']
        tpos, tvel = st['target_position'], st['target_velocity']
        if self.kind == 'pn':
            from d2dtracker_interception.guidance import pn_velocity_command
            v = pn_velocity_command(ipos, ivel, tpos, tvel, self.v_max,
                                    N=self.nav_gain, dt=self.dt,
                                    closing_speed=self.closing_speed)
        else:
            rel = np.asarray(tpos, float) - np.asarray(ipos, float)
            n = float(np.linalg.norm(rel))
            v = rel / n * self.v_max if n > 1e-6 else np.zeros(3)
        return np.clip(np.asarray(v, float) / self.v_max, -1.0, 1.0), None


def main(args=None):
    p = argparse.ArgumentParser(description='AeroGym interception evaluator')
    p.add_argument('--model', default=None,
                   help='SB3 .zip (required for --policy model)')
    p.add_argument('--policy', default='model', choices=['model', 'pn', 'pursuit'])
    p.add_argument('--episodes', type=int, default=10)
    p.add_argument('--target-policy', default='static')
    p.add_argument('--target-speed', type=float, default=0.5,
                   help='moving-target speed (match the training stage)')
    p.add_argument('--speed-factor', type=float, default=4.0)
    p.add_argument('--rank', type=int, default=2,
                   help='pair rank for the eval stack (PX4 instances 2r/2r+1; '
                        'avoid training ranks)')
    p.add_argument('--domain-base', type=int, default=100,
                   help='ROS_DOMAIN_ID of the stack = domain_base + 2*rank')
    p.add_argument('--run-id', default='icpt_eval',
                   help='names the stack (GZ_PARTITION aerogym_<run-id>_<2*rank>)')
    p.add_argument('--seed', type=int, default=100)
    p.add_argument('--device', default='cpu', help='torch device for the policy')
    p.add_argument('--csv', default=None, help='append per-episode rows here')
    p.add_argument('--traj-dir', default=None, help='save per-episode .npz here')
    p.add_argument('--label', default=None,
                   help='policy label for the CSV (default: model file or baseline)')
    p.add_argument('--cbf', action='store_true',
                   help='filter actions with the CBF safety filter')
    p.add_argument('--cbf-z-min', type=float, default=1.5)
    p.add_argument('--cbf-alpha', type=float, default=1.0)
    args = p.parse_args(args)
    if args.policy == 'model' and not args.model:
        p.error('--model is required for --policy model')

    eval_cfg = {'target_policy': args.target_policy}
    if args.target_policy in ('constant_velocity', 'evasive'):
        eval_cfg['target_policy_params'] = {'velocity': [args.target_speed, 0.0, 0.0]}

    from . import task  # noqa: F401
    from .worker import build_intercept_worker_env
    env = build_intercept_worker_env(
        args.rank, eval_cfg, run_id=args.run_id, speed_factor=args.speed_factor,
        domain_base=args.domain_base, stagger=0.0)

    if args.policy == 'model':
        from stable_baselines3 import PPO, SAC
        model = None
        for cls in (SAC, PPO):
            try:
                model = cls.load(args.model, device=args.device)
                break
            except Exception:
                continue
        if model is None:
            raise SystemExit(f'could not load {args.model}')
        label = args.label or os.path.splitext(os.path.basename(args.model))[0]
    else:
        model = ScriptedPolicy(args.policy, env)
        label = args.label or args.policy
    cbf = None
    if args.cbf:
        from d2dtracker_interception.safety import CBFVelocityFilter
        cbf = CBFVelocityFilter(
            z_min=args.cbf_z_min, z_max=float(env.cfg['arena_alt_max']) - 2.0,
            fence_radius=float(env.cfg['arena_horiz']) - 5.0, alpha=args.cbf_alpha)
        v_max = float(env.cfg.get('v_max', 8.0))
        if not args.label:
            label += '+cbf'

    writer = None
    if args.csv:
        os.makedirs(os.path.dirname(os.path.abspath(args.csv)), exist_ok=True)
        new = not os.path.exists(args.csv)
        fcsv = open(args.csv, 'a', newline='')
        writer = csv.DictWriter(fcsv, fieldnames=CSV_FIELDS)
        if new:
            writer.writeheader()
    if args.traj_dir:
        os.makedirs(args.traj_dir, exist_ok=True)

    captures, times, min_ranges = 0, [], []
    for ep in range(args.episodes):
        seed = args.seed + ep
        obs, _ = env.reset(seed=seed)
        done = trunc = False
        min_rng, min_alt, n_active = float('inf'), float('inf'), 0
        rows, steps, info = [], 0, {}
        while not (done or trunc):
            action, _ = model.predict(obs, deterministic=True)
            if cbf is not None:
                v, active = cbf.apply(env._state()['position'],
                                      np.clip(np.asarray(action, float), -1, 1) * v_max)
                action = np.clip(v / v_max, -1.0, 1.0).astype(np.float32)
                n_active += int(active)
            obs, r, done, trunc, info = env.step(action)
            steps += 1
            st = env._state()
            min_rng = min(min_rng, float(np.linalg.norm(
                st['target_position'] - st['position'])))
            min_alt = min(min_alt, float(st['position'][2]))
            if args.traj_dir:
                rows.append(np.concatenate([[env._t], st['position'],
                                            st['target_position']]))
        outcome = info.get('outcome', 'timeout')
        if outcome == 'capture':
            captures += 1
            times.append(env._t)
        min_ranges.append(min_rng)
        print(f'  ep {ep:2d}: outcome={outcome:13s} t={env._t:6.2f}s '
              f'min_range={min_rng:6.2f} m', flush=True)
        if writer is not None:
            writer.writerow({
                'policy': label, 'model': args.model or '', 'episode': ep,
                'target_policy': args.target_policy, 'seed': seed,
                'target_speed': (args.target_speed
                                 if args.target_policy != 'static' else 0.0),
                'outcome': outcome, 't_s': f'{env._t:.2f}',
                'min_range_m': f'{min_rng:.3f}', 'steps': steps,
                'min_alt_m': f'{min_alt:.3f}',
                'cbf_active_frac': (f'{n_active / max(steps, 1):.3f}'
                                    if cbf is not None else '')})
            fcsv.flush()
        if args.traj_dir and rows:
            name = (f'{label}__{args.target_policy}_{args.target_speed:g}'
                    f'__ep{ep:02d}.npz')
            np.savez(os.path.join(args.traj_dir, name), traj=np.array(rows),
                     outcome=outcome, seed=seed)

    print(f'\n==== intercept eval ({args.episodes} eps, policy={label}, '
          f'target={args.target_policy}) ====')
    print(f'  capture rate : {captures}/{args.episodes}')
    if times:
        print(f'  mean t_capture: {np.mean(times):.2f} s')
    print(f'  mean min_range: {np.mean(min_ranges):.2f} m')
    env.close()


if __name__ == '__main__':
    main()
