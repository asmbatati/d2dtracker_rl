#!/usr/bin/env python3
"""Plot the interception curriculum's learning curves from the SB3 train logs.

Parses the stdout tables Stable-Baselines3 prints during ``train_intercept``
(``rollout/ep_rew_mean``, ``rollout/ep_len_mean``, ``time/total_timesteps``)
and draws the stages end to end on one cumulative timestep axis, marking the
checkpoint each stage kept.

    python3 scripts/plot_training.py --out docs/figures/learning_curves.png

Default stages are the June 2026 curriculum (logs under ``checkpoints/``, not
in git): from-scratch static (v4) -> static refine -> constant velocity ->
evasive.
"""
import argparse
import os
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CK = os.path.join(PKG, 'checkpoints')
STAGES = [  # label, log, kept checkpoint (timesteps)
    ('static (from scratch)', 'intercept_v4_static/train_static_v4.log', 120000),
    ('static refine', 'intercept/train_refine.log', 40000),
    ('constant velocity', 'intercept/train_cv.log', 40000),
    ('evasive', 'intercept/train_evasive.log', 40000),
]
SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100']
INK, INK2, GRID = '#0b0b0b', '#52514e', '#e4e3df'
ROW = re.compile(r'^\|\s+(\w+)\s+\|\s+([-+0-9.eE]+)\s+\|')


def parse(path):
    """Return a list of (timesteps, ep_rew_mean, ep_len_mean)."""
    rows, cur = [], {}
    with open(path) as f:
        for line in f:
            m = ROW.match(line.strip())
            if m:
                cur[m.group(1)] = float(m.group(2))
            elif line.startswith('-----') and cur:
                if 'total_timesteps' in cur and 'ep_rew_mean' in cur:
                    rows.append((cur['total_timesteps'], cur['ep_rew_mean'],
                                 cur.get('ep_len_mean', float('nan'))))
                cur = {}
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--out', default=os.path.join(PKG, 'docs', 'figures',
                                                 'learning_curves.png'))
    a = p.parse_args()
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    fig, (ax_r, ax_l) = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
    offset = 0
    kept_x = []
    for i, (label, log, kept) in enumerate(STAGES):
        rows = parse(os.path.join(CK, log))
        if not rows:
            print(f'no rows in {log}')
            continue
        t = [offset + r[0] / 1000 for r in rows]
        ax_r.plot(t, [r[1] for r in rows], color=SERIES[i], linewidth=1.6, label=label)
        ax_l.plot(t, [r[2] for r in rows], color=SERIES[i], linewidth=1.6)
        for ax in (ax_r, ax_l):
            ax.axvline(offset + kept / 1000, color=SERIES[i], linewidth=0.8,
                       linestyle='--')
        kept_x.append(offset + kept / 1000)
        offset = t[-1]
        print(f'{label}: {len(rows)} points, {rows[-1][0]:.0f} steps')
    top = ax_r.get_ylim()[1]
    for x in kept_x:
        ax_r.annotate(' kept', (x, top), fontsize=7, color=INK2, ha='left', va='top')
    for ax in (ax_r, ax_l):
        ax.spines[['top', 'right']].set_visible(False)
        ax.spines[['left', 'bottom']].set_color(INK2)
        ax.tick_params(colors=INK2, labelsize=8)
        ax.grid(axis='y', color=GRID, linewidth=0.8)
    ax_r.set_ylabel('episode reward\n(stochastic, rolling mean)', fontsize=8, color=INK)
    ax_l.set_ylabel('episode length (steps)', fontsize=8, color=INK)
    ax_l.set_xlabel('cumulative training steps (thousands), stages end to end',
                    fontsize=8, color=INK)
    ax_r.legend(fontsize=7, frameon=False, ncol=4, loc='lower right')
    fig.tight_layout()
    fig.savefig(a.out, dpi=200)
    print(f'-> {a.out}')


if __name__ == '__main__':
    main()
