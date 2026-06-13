"""Worker-side construction for interception training (managed two-vehicle sim).

Mirrors aerogym's ``vec/worker_env.py`` (same isolation-before-rclpy order,
boot retry, identifier threading), but boots an :class:`InterceptSimInstance`
and an :class:`InterceptEnv`. Each logical worker rank consumes TWO PX4
instances (2r, 2r+1) — keep pair ranks in 0..4.
"""
import time


def build_intercept_worker_env(rank, env_cfg, run_id='intercept',
                               speed_factor=1.0, domain_base=100,
                               boot_timeout=300.0, stagger=5.0):
    import os

    from .sim import InterceptSimInstance

    if stagger and rank:
        time.sleep(rank * stagger)

    env_cfg = dict(env_cfg or {})
    sim = InterceptSimInstance(
        rank=rank, run_id=run_id, speed_factor=speed_factor,
        domain_base=domain_base,
        target_spawn=tuple(env_cfg.get('target_spawn', [10.0, 0.0, 0.0]))[:2]
        + (0.2,))
    os.environ.update(sim.child_env)   # BEFORE rclpy.init
    sim.boot()

    import rclpy
    if not rclpy.ok():
        rclpy.init()

    env_cfg['world'] = sim.world
    env_cfg['entity'] = sim.entity
    env_cfg['target_entity'] = sim.target_entity

    from .env import InterceptEnv
    env = InterceptEnv(config=env_cfg, sim=sim)

    for attempt in range(3):
        deadline = time.monotonic() + boot_timeout
        while time.monotonic() < deadline:
            if (env.ros.sim_time() is not None and env.ros.connected()
                    and env.target_ros.connected()):
                return env
            time.sleep(1.0)
        if attempt < 2:
            sim.restart()
    sim.close()
    raise RuntimeError(f'intercept worker {rank}: stack not ready after '
                       f'3 boots x {boot_timeout}s')


def make_intercept_vec_env(n_envs, env_cfg, run_id='intercept',
                           speed_factor=1.0, domain_base=100, seed=0,
                           rank_offset=0):
    """SB3 SubprocVecEnv of interception envs (spawn; one 2-vehicle stack each)."""
    from stable_baselines3.common.vec_env import SubprocVecEnv

    def thunk(i):
        def _t():
            from stable_baselines3.common.monitor import Monitor
            env = build_intercept_worker_env(
                rank_offset + i, env_cfg, run_id=run_id,
                speed_factor=speed_factor, domain_base=domain_base)
            env.reset(seed=seed + i)
            return Monitor(env)
        return _t

    return SubprocVecEnv([thunk(i) for i in range(n_envs)],
                         start_method='spawn')
