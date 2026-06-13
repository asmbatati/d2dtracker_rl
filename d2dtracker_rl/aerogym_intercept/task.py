"""Interception task plugin for the AeroGym-PX4 environment.

Registers ``intercept`` into aerogym's task registry — from THIS package, so
the playground itself stays interception-free. The task consumes the enriched
state dict produced by :class:`InterceptEnv` (which adds ``target_position`` /
``target_velocity`` in the common world frame).

Reward design carries AeroGym's measured shaping lessons:
* capture is a large terminal bonus; crash/out-of-bounds cost 1000 at the env
  level (absorbing exits must never be cheaper than flying badly),
* closing-rate shaping plus a near-field precision bell around the target so
  the gradient never flattens on final approach.
"""
import numpy as np

from aerogym_px4.tasks.base import Task, register


@register('intercept')
class InterceptTask(Task):
    """Chase and capture a maneuvering target (second PX4 vehicle)."""

    obs_dim = 10  # rel_pos/ps (3) + rel_vel/vs (3) + range/ps (1) + LOS (3)

    def configure(self, cfg):
        self.spawn_center = np.asarray(
            cfg.get('interceptor_spawn', [0.0, 0.0, 5.0]), dtype=float)
        self.spawn_jitter = float(cfg.get('spawn_jitter', 2.0))
        self.capture_radius = float(cfg.get('capture_radius', 1.0))
        self.pos_scale = float(cfg.get('pos_scale', 30.0))
        self.vel_scale = float(cfg.get('vel_scale', 10.0))
        self.w_close = float(cfg.get('w_close', 1.0))
        self.w_range = float(cfg.get('w_range', 0.05))
        self.w_align = float(cfg.get('w_align', 0.2))
        self.w_effort = float(cfg.get('w_effort', 0.01))
        self.w_prec = float(cfg.get('w_prec', 2.0))
        self.prec_sigma = float(cfg.get('prec_sigma', 3.0))
        self.r_capture = float(cfg.get('r_capture', 1000.0))
        self._prev_range = None

    def reset(self, rng):
        self._prev_range = None
        jitter = rng.uniform(-self.spawn_jitter, self.spawn_jitter, 3)
        jitter[2] = abs(jitter[2]) * 0.5
        return self.spawn_center + jitter

    @staticmethod
    def _rel(state):
        rel_pos = state['target_position'] - state['position']
        rel_vel = state['target_velocity'] - state['velocity']
        rng = float(np.linalg.norm(rel_pos))
        return rel_pos, rel_vel, rng

    def task_obs(self, state, t):
        rel_pos, rel_vel, rng = self._rel(state)
        los = rel_pos / rng if rng > 1e-6 else np.zeros(3)
        return np.concatenate([
            rel_pos / self.pos_scale,
            rel_vel / self.vel_scale,
            [rng / self.pos_scale],
            los,
        ]).astype(np.float32)

    def reward(self, state, action, prev_state, t):
        rel_pos, _, rng = self._rel(state)
        r = 0.0
        if self._prev_range is not None:
            r += self.w_close * (self._prev_range - rng)
        self._prev_range = rng
        r -= self.w_range * rng
        speed = float(np.linalg.norm(state['velocity']))
        if speed > 1e-3 and rng > 1e-3:
            r += self.w_align * float(
                np.dot(state['velocity'], rel_pos) / (speed * rng))
        r += self.w_prec * float(np.exp(-(rng / self.prec_sigma) ** 2))
        r -= self.w_effort * float(np.sum(np.square(action)))
        if rng <= self.capture_radius:
            r += self.r_capture
        return r

    def termination(self, state, t):
        _, _, rng = self._rel(state)
        if rng <= self.capture_radius:
            return True, 'capture'
        return False, None
