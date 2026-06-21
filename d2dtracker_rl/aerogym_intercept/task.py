"""Interception task plugin for the AeroGym-PX4 environment.

Registers ``intercept`` into aerogym's task registry — from THIS package, so
the playground itself stays interception-free. The task consumes the enriched
state dict produced by :class:`InterceptEnv` (which adds ``target_position`` /
``target_velocity`` in the common world frame).

Reward design carries AeroGym's measured shaping lessons:
* capture is a large terminal bonus; crash/out-of-bounds cost 1000 at the env
  level (absorbing exits must never be cheaper than flying badly),
* closing-rate shaping plus a near-field precision bell around the target so
  the gradient never flattens on final approach,
* a DENSE altitude-floor penalty above the hard crash line, and a small alive
  bonus. These were added after the first curriculum produced a policy that
  looked fine in (stochastic) training — ep_len 400, no crashes — yet sank into
  the ground when evaluated deterministically: SAC's exploration noise had been
  holding it aloft, and nothing in the reward made *approaching* the crash line
  costly until the one-shot -1000 at the boundary (too sparse for the value
  function to learn under exploration). The floor penalty gives a continuous
  upward gradient; the alive bonus keeps the living baseline non-negative so the
  agent is never tempted to crash early just to stop bleeding step penalties.
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
        # w_range gives a constant inward pull at all distances (closing-rate is
        # ~0 once the craft station-keeps). prec bell is the near-field pull.
        self.w_range = float(cfg.get('w_range', 0.1))
        self.w_align = float(cfg.get('w_align', 0.2))
        self.w_effort = float(cfg.get('w_effort', 0.01))
        # Wide, strong precision bell. A narrow bell (sigma 2.5) left the policy
        # parked at ~3.8 m — OUTSIDE the bell, where its gradient is flat, so
        # nothing pulled it the final few metres into the <1 m capture gate
        # (eval plateaued at 0/6 capture, mean min_range 3.78 m). sigma 4.0 puts
        # a meaningful, monotonically rising pull from ~5 m all the way to 0.
        self.w_prec = float(cfg.get('w_prec', 6.0))
        self.prec_sigma = float(cfg.get('prec_sigma', 4.0))
        self.r_capture = float(cfg.get('r_capture', 1000.0))
        # altitude safety: penalize flying below a floor set well above the hard
        # crash line, so the deterministic policy keeps a margin instead of
        # sinking to a crash. floor is in metres AGL (interceptor EKF frame).
        self.alt_floor = float(cfg.get('alt_floor', 2.5))
        self.w_alt = float(cfg.get('w_alt', 2.0))
        # arena-boundary safety: a DENSE penalty as the craft nears the walls /
        # ceiling, mirroring the altitude floor. Without it the only signal to
        # stay in the arena is the sparse one-shot -1000 out-of-bounds, too late
        # for the value function — the 40k policy closed to ~3 m then overshot
        # straight out of bounds. Margin (m) inside each boundary where it bites.
        self.arena_horiz = float(cfg.get('arena_horiz', 60.0))
        self.arena_alt_max = float(cfg.get('arena_alt_max', 30.0))
        self.bound_margin = float(cfg.get('bound_margin', 10.0))
        self.w_bound = float(cfg.get('w_bound', 2.0))
        # small per-step alive bonus: keeps the living baseline non-negative
        # (range penalty + effort would otherwise make every step a small loss).
        self.w_alive = float(cfg.get('w_alive', 0.05))
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
        # closing progress: telescopes over the episode to net range reduction
        if self._prev_range is not None:
            r += self.w_close * (self._prev_range - rng)
        self._prev_range = rng
        # steady pressure to be close (closing alone is 0 at a stable trail dist)
        r -= self.w_range * rng
        # velocity heading aligned with the line of sight to the target
        speed = float(np.linalg.norm(state['velocity']))
        if speed > 1e-3 and rng > 1e-3:
            r += self.w_align * float(
                np.dot(state['velocity'], rel_pos) / (speed * rng))
        # near-field precision bell (strong, tight: dominates the final approach)
        r += self.w_prec * float(np.exp(-(rng / self.prec_sigma) ** 2))
        # DENSE altitude-floor penalty: continuous upward gradient above the
        # crash line so the deterministic policy never learns to sink into it
        pos = state['position']
        alt = float(pos[2])
        if alt < self.alt_floor:
            r -= self.w_alt * (self.alt_floor - alt)
        # DENSE arena-boundary penalty: ramps up over the last bound_margin
        # metres before each wall / the ceiling, so the policy turns back instead
        # of overshooting the target straight out of the arena
        horiz = float(np.linalg.norm(pos[:2]))
        if horiz > self.arena_horiz - self.bound_margin:
            r -= self.w_bound * (horiz - (self.arena_horiz - self.bound_margin))
        if alt > self.arena_alt_max - self.bound_margin:
            r -= self.w_bound * (alt - (self.arena_alt_max - self.bound_margin))
        # alive bonus + effort regularization
        r += self.w_alive
        r -= self.w_effort * float(np.sum(np.square(action)))
        if rng <= self.capture_radius:
            r += self.r_capture
        return r

    def termination(self, state, t):
        _, _, rng = self._rel(state)
        if rng <= self.capture_radius:
            return True, 'capture'
        return False, None
