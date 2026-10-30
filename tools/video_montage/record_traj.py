import os, numpy as np
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
from d2dtracker_rl.aerogym_intercept import task  # noqa
from d2dtracker_rl.aerogym_intercept.worker import build_intercept_worker_env
from stable_baselines3 import SAC

PKG = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CKPT = os.environ.get('CKPT_DIR', os.path.join(PKG, 'checkpoints', 'intercept'))
ARCH = CKPT + '_v4_static'
OUT = os.path.join(os.environ.get('MONTAGE_DIR', 'montage_out'), 'traj'); os.makedirs(OUT, exist_ok=True)
def rp(p): return p if os.path.isfile(p) else os.path.join(ARCH, os.path.basename(p))

GROUPS = [
  ('static', 0.0, [('sac_intercept_static_20000_steps.zip', '20k steps  -  learning to fly', 2),
                   ('sac_intercept_static_60000_steps.zip', '60k steps  -  stable tracking', 1),
                   ('sac_intercept_static_100000_steps.zip', '100k steps  -  closing in', 1),
                   ('model_static_refine_40k.zip', 'refined  -  CAPTURE', 2)]),
  ('constant_velocity', 0.5, [('model_cv_40k.zip', 'moving target  -  10/10', 2)]),
  ('evasive', 0.5, [('model_evasive_40k.zip', 'evasive target  -  9/10', 2)]),
]
for tp, spd, models in GROUPS:
    cfg = {'target_policy': tp}
    if tp != 'static':
        cfg['target_policy_params'] = {'velocity': [spd, 0.0, 0.0]}
    env = build_intercept_worker_env(3, cfg, run_id='icpt_rec', speed_factor=4.0, stagger=0.0)
    for fname, caption, eps in models:
        mp = rp(os.path.join(CKPT, fname)); model = SAC.load(mp)
        rows = []; outs = []
        for ep in range(eps):
            obs, _ = env.reset(seed=ep)
            done = trunc = False; n = 0
            while not (done or trunc) and n < 450:
                a, _ = model.predict(obs, deterministic=True)
                obs, r, done, trunc, info = env.step(a); n += 1
                st = env._state()
                tv = st['target_velocity']
                rows.append(np.concatenate([st['position'], st['quaternion'],
                                            st['target_position'], tv[:2]]))
            outs.append(info.get('outcome'))
            rows.append(np.full(12, np.nan))  # episode separator
        slug = fname.replace('.zip', '')
        np.savez(f'{OUT}/{tp}__{slug}.npz', frames=np.array(rows, float),
                 caption=caption, outcomes=outs)
        print(f'[{tp}/{slug}] eps={eps} outcomes={outs} rows={len(rows)}', flush=True)
    env.close()
print('RECORD_DONE', flush=True)
