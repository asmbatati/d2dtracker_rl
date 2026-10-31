#!/usr/bin/env python3
"""Summarize an eval_matrix.sh run: tables and figures for the RL paper.

Reads ``<run>/episodes.csv`` (one row per deterministic episode) and writes to
``<run>/report/``:

* ``summary.csv`` / ``summary.md`` - per (target policy, speed, policy):
  episodes, captures, capture rate with a Wilson 95 % interval, median capture
  time, median closest approach, crashes / out-of-bounds, lowest altitude and,
  for ``+cbf`` rows, the fraction of steps the filter changed the command;
* ``capture_vs_speed.png`` - capture rate against target speed, one panel per
  moving-target policy, one line per policy (when several speeds were run);
* ``capture_by_target.png`` - capture rate per policy at each target policy
  (first speed).

    python3 scripts/analyze_eval.py ~/rl_eval/<run>
"""
import argparse
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300',
          '#6250d6', '#e34948']
INK, INK2, GRID = '#0b0b0b', '#52514e', '#e4e3df'


def wilson(k, n, z=1.96):
    if n == 0:
        return (float('nan'),) * 3
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return p, max(0.0, c - h), min(1.0, c + h)


def style(ax):
    ax.spines[['top', 'right']].set_visible(False)
    ax.spines[['left', 'bottom']].set_color(INK2)
    ax.tick_params(colors=INK2, labelsize=8)
    ax.grid(axis='y', color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


INFRA = ['sim_stalled', 'state_invalid']   # stack faults, not policy outcomes
TARGETS = ['static', 'constant_velocity', 'evasive']
BASELINES = ['pn', 'pn_int', 'pursuit']
STAGES = [('static',), ('cv', 'constant_velocity'), ('evasive',)]   # model names


def policy_key(pol):
    """Models in curriculum order, then the baselines, then the +cbf variants."""
    base = pol.removesuffix('+cbf')
    if base in BASELINES:
        rank = (1, BASELINES.index(base))
    else:
        rank = (0, next((i for i, words in enumerate(STAGES)
                         if any(w in base for w in words)), len(STAGES)))
    return (pol.endswith('+cbf'),) + rank + (base,)


FAMILIES = ['RL policy', 'scripted guidance', 'behind the CBF filter']


def family(pol):
    """Bar colour: what kind of controller, since each bar is labelled anyway."""
    if pol.endswith('+cbf'):
        return 2
    return 1 if pol in BASELINES else 0


def row_key(r):
    tp = r['target_policy']
    return (TARGETS.index(tp) if tp in TARGETS else len(TARGETS), r['target_speed'],
            policy_key(r['policy']))


def summarize(df):
    rows = []
    keys = ['target_policy', 'target_speed', 'policy']
    for (tp, spd, pol), g in df.groupby(keys, sort=False):
        n = len(g)
        cap = g[g.outcome == 'capture']
        p, lo, hi = wilson(len(cap), n)
        act = (pd.to_numeric(g['cbf_active_frac'], errors='coerce')
               if 'cbf_active_frac' in g else pd.Series(dtype=float))
        rows.append({
            'target_policy': tp, 'target_speed': spd, 'policy': pol,
            'episodes': n, 'captures': len(cap), 'capture_rate': p,
            'ci_lo': lo, 'ci_hi': hi,
            't_capture_median_s': cap.t_s.median() if len(cap) else np.nan,
            'min_range_median_m': g.min_range_m.median(),
            'crash': int(g.outcome.str.contains('crash').sum()),
            'out_of_bounds': int(g.outcome.str.contains('oob|bound').sum()),
            'min_alt_m': (pd.to_numeric(g['min_alt_m'], errors='coerce').min()
                          if 'min_alt_m' in g else np.nan),
            'cbf_active_mean': act.mean() if act.notna().any() else np.nan,
            'step_dt_median_s': (pd.to_numeric(g['step_dt_s'], errors='coerce').median()
                                 if 'step_dt_s' in g else np.nan),
        })
    return pd.DataFrame(sorted(rows, key=row_key))   # CSV order follows re-runs


def write_md(s, path):
    lines = ['| target | speed (m/s) | policy | captures | capture rate (95 % CI) | '
             'median t_capture (s) | median closest (m) | crash / oob | '
             'lowest alt (m) | CBF active | s/step |',
             '|---|---|---|---|---|---|---|---|---|---|---|']
    for r in s.itertuples():
        t = '—' if np.isnan(r.t_capture_median_s) else f'{r.t_capture_median_s:.1f}'
        alt = '—' if np.isnan(r.min_alt_m) else f'{r.min_alt_m:.2f}'
        act = '' if np.isnan(r.cbf_active_mean) else f'{r.cbf_active_mean:.0%}'
        lines.append(
            f'| {r.target_policy} | {r.target_speed:g} | `{r.policy}` | '
            f'{r.captures}/{r.episodes} | {r.capture_rate:.0%} '
            f'({r.ci_lo:.0%}–{r.ci_hi:.0%}) | {t} | {r.min_range_median_m:.2f} | '
            f'{r.crash} / {r.out_of_bounds} | {alt} | {act} | '
            f'{"—" if np.isnan(r.step_dt_median_s) else f"{r.step_dt_median_s:.3f}"} |')
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def fig_speed(s, policies, path):
    moving = [tp for tp in dict.fromkeys(s.target_policy) if tp != 'static']
    if not moving or s[s.target_policy.isin(moving)].target_speed.nunique() < 2:
        return False
    fig, axes = plt.subplots(1, len(moving), figsize=(4.2 * len(moving), 3.4),
                             sharey=True, squeeze=False)
    for ax, tp in zip(axes[0], moving):
        sub = s[s.target_policy == tp]
        for i, pol in enumerate(policies):
            g = sub[sub.policy == pol].sort_values('target_speed')
            if g.empty:
                continue
            ax.plot(g.target_speed, g.capture_rate, color=SERIES[i % len(SERIES)],
                    linewidth=2, marker='o', markersize=5, label=pol)
        ax.set_title(tp.replace('_', ' '), fontsize=9, color=INK)
        ax.set_xlabel('target speed (m/s)', fontsize=8, color=INK)
        ax.set_ylim(-0.05, 1.05)
        style(ax)
    axes[0][0].set_ylabel('capture rate', fontsize=8, color=INK)
    axes[0][-1].legend(fontsize=7, frameon=False, loc='lower left')
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)
    return True


def fig_targets(s, policies, path):
    targets = list(dict.fromkeys(s.target_policy))
    fig, axes = plt.subplots(1, len(targets), figsize=(4.6 * len(targets), 3.6),
                             sharey=True, squeeze=False)
    for ax, tp in zip(axes[0], targets):
        sub = s[s.target_policy == tp]
        sub = sub[sub.target_speed == sub.target_speed.min()].set_index('policy')
        for i, pol in enumerate(policies):
            if pol not in sub.index:
                continue
            r = sub.loc[pol]
            ax.bar(i, r.capture_rate, width=0.6, color=SERIES[family(pol)],
                   edgecolor='white', linewidth=2)
            ax.errorbar(i, r.capture_rate,
                        yerr=[[max(0.0, r.capture_rate - r.ci_lo)],
                              [max(0.0, r.ci_hi - r.capture_rate)]],
                        color=INK2, capsize=3, linewidth=1)
            ax.text(i, min(r.ci_hi, 1.0) + 0.03, f'{r.captures}/{r.episodes}',
                    ha='center', fontsize=6, color=INK)
        ax.set_xticks(range(len(policies)))
        ax.set_xticklabels(policies, rotation=35, ha='right', fontsize=7)
        spd = sub.target_speed.iloc[0] if len(sub) else 0
        ax.set_title(f'{tp.replace("_", " ")}'
                     + (f' ({spd:g} m/s)' if tp != 'static' else ''),
                     fontsize=9, color=INK)
        ax.set_ylim(0, 1.15)
        style(ax)
    axes[0][0].set_ylabel('capture rate (95 % Wilson CI)', fontsize=8, color=INK)
    shown = sorted({family(p) for p in policies})
    fig.legend([plt.Rectangle((0, 0), 1, 1, color=SERIES[k]) for k in shown],
               [FAMILIES[k] for k in shown], fontsize=8, frameon=False,
               loc='upper center', ncol=len(shown))
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(path, dpi=200)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('run', help='eval_matrix.sh output dir (holds episodes.csv)')
    p.add_argument('--max-step-dt', type=float, default=None,
                   help='exclude episodes whose mean sim seconds per step '
                        '(step_dt_s) exceed this')
    a = p.parse_args()
    df = pd.read_csv(os.path.join(a.run, 'episodes.csv'))
    bad_reset = (df['target_ready'] == 0) if 'target_ready' in df else False
    n_infra = int((df.outcome.isin(INFRA) | bad_reset).sum())
    df = df[~(df.outcome.isin(INFRA) | bad_reset)]   # stack faults / target not at start
    n_slow = 0
    if a.max_step_dt and 'step_dt_s' in df:   # control period stretched by load
        slow = pd.to_numeric(df.step_dt_s, errors='coerce') > a.max_step_dt
        n_slow = int(slow.sum())
        df = df[~slow]
    out = os.path.join(a.run, 'report')
    os.makedirs(out, exist_ok=True)
    policies = sorted(dict.fromkeys(df.policy), key=policy_key)
    s = summarize(df)
    s.to_csv(os.path.join(out, 'summary.csv'), index=False, float_format='%.3f')
    write_md(s, os.path.join(out, 'summary.md'))
    fig_targets(s, policies, os.path.join(out, 'capture_by_target.png'))
    fig_speed(s, policies, os.path.join(out, 'capture_vs_speed.png'))
    print(open(os.path.join(out, 'summary.md')).read())
    print(f'{n_infra} infrastructure-fault and {n_slow} slow-step episodes '
          f'excluded; figures -> {out}')


if __name__ == '__main__':
    main()
