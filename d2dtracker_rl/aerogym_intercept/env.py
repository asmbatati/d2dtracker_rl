"""Interception environment: AeroGym's BasePX4Env + a second (target) vehicle.

The interceptor side is pure AeroGym (sim-time stepping, rescue ladder, NaN
guards, action interfaces). This subclass adds:

* a second :class:`VehicleRosInterface` on the ``target`` namespace,
* the enriched state dict (``target_position`` / ``target_velocity`` in the
  common world frame, via the target's spawn offset — each PX4's EKF origin is
  its own spawn, the convention proven across the interception workspace),
* a scripted target policy (static / constant-velocity / evasive — the
  curriculum axis) driven once per control tick,
* target episode reset: teleport + re-arm/OFFBOARD hold.
"""
import time

import numpy as np

from aerogym_px4.core.base_env import BasePX4Env
from aerogym_px4.core.px4_reset import gz_set_entity_pose
from aerogym_px4.core.ros_interface import VehicleRosInterface

from ..target_policy import make_target_policy

INTERCEPT_DEFAULTS = {
    'task': 'intercept',
    'target_ns': 'target',
    'target_entity': 'x500_1',
    'target_spawn': [10.0, 0.0, 0.0],     # world offset of the target's EKF origin
    'target_altitude': 5.0,
    'target_policy': 'static',
    'target_policy_params': {},
    'target_jitter': 3.0,
    'episode_steps': 400,
    'arena_horiz': 60.0,
    'v_max': 8.0,
    'r_crash': 1000.0,
    'r_oob': 1000.0,
}


class InterceptEnv(BasePX4Env):
    """Two-vehicle interception env on the AeroGym core."""

    def __init__(self, config=None, ros=None, sim=None):
        cfg = dict(INTERCEPT_DEFAULTS)
        if config:
            cfg.update(config)
        cfg['task'] = 'intercept'
        super().__init__(config=cfg, ros=ros, sim=sim)
        self.target_ros = VehicleRosInterface(
            self.cfg['target_ns'], node_name='aerogym_target_interface')
        self._tspawn = np.asarray(self.cfg['target_spawn'], dtype=float)
        self._target_policy = None
        self._target_start = self._tspawn + [0, 0, self.cfg['target_altitude']]

    # --- state ---------------------------------------------------------------
    def _state(self):
        state = super()._state()
        if state is None:
            return None
        todom = self.target_ros.odom()
        if todom is not None:
            p, v = todom.pose.pose.position, todom.twist.twist.linear
            state['target_position'] = np.array([p.x, p.y, p.z]) + self._tspawn
            state['target_velocity'] = np.array([v.x, v.y, v.z])
        else:
            state['target_position'] = np.asarray(self._target_start, dtype=float)
            state['target_velocity'] = np.zeros(3)
        return state

    # --- target drive ----------------------------------------------------------
    def _drive_target(self):
        if self._target_policy is None:
            return
        x, y, z = self._target_policy.setpoint(self._t)
        # policy works in world frame; convert to the target's odom frame
        self.target_ros.publish_position_target(
            x - self._tspawn[0], y - self._tspawn[1], z - self._tspawn[2])
        st = self.target_ros.state()
        if st.mode != 'OFFBOARD':
            self.target_ros.set_mode('OFFBOARD')
        if not st.armed:
            self.target_ros.arm(True)

    def step(self, action):
        self._drive_target()
        return super().step(action)

    # --- target recovery -------------------------------------------------------
    def _target_airborne(self):
        todom = self.target_ros.odom()
        return (todom is not None and todom.pose.pose.position.z > 1.0
                and self.target_ros.state().armed)

    def _target_near(self, start, tol=2.5):
        todom = self.target_ros.odom()
        if todom is None:
            return False
        p = todom.pose.pose.position
        pos = np.array([p.x, p.y, p.z]) + self._tspawn   # world frame
        return float(np.linalg.norm(pos - np.asarray(start, float))) < tol

    def _cosettle(self, spawn, target_start, max_sim=15.0):
        """Bring BOTH vehicles to their starts, commanding them every tick.

        Run AFTER the interceptor's own reset. Capturing the target makes the two
        physical gz models collide, which can knock the target to the ground; the
        old airborne-only teleport could not recover it (observed: a grounded
        target at [-22.9, -5.4, 0.2] -> unwinnable episode + buffer poison). And
        because both PX4s need continuous OFFBOARD setpoints, the target must be
        re-flown to altitude WHILE the interceptor is still streamed each tick, or
        one of them drops OFFBOARD and sinks. So: clear a grounded target (level
        it low, disarm to clear the lock), then stream interceptor->spawn and
        target->start together until both are at their starts and airborne.
        """
        spawn = np.asarray(spawn, dtype=float)
        target_start = np.asarray(target_start, dtype=float)
        extra = self.sim.child_env if self.sim is not None else None
        if not self._target_airborne():
            gz_set_entity_pose(self.cfg['world'], self.cfg['target_entity'],
                               float(target_start[0]), float(target_start[1]), 0.3,
                               extra_env=extra)
            self.target_ros.arm(False)                   # clear crash/failsafe
            self._wait_tick()
        elif not self._target_near(target_start, tol=3.0):
            gz_set_entity_pose(self.cfg['world'], self.cfg['target_entity'],
                               float(target_start[0]), float(target_start[1]),
                               float(target_start[2]), extra_env=extra)
        thold = target_start - self._tspawn              # target's own EKF frame
        t_end = (self.ros.sim_time() or 0.0) + max_sim
        deadline = time.monotonic() + 2 * max_sim + 10.0
        while True:
            if (self._ready(spawn) and self._target_near(target_start)
                    and self._target_airborne()):
                return
            now = self.ros.sim_time()
            if (self.use_sim_time and now is not None and now >= t_end) \
                    or time.monotonic() >= deadline:
                return
            self.ros.publish_position_target(*spawn)     # hold interceptor
            si = self.ros.state()
            if si.mode != 'OFFBOARD':
                self.ros.set_mode('OFFBOARD')
            if not si.armed:
                self.ros.arm(True)
            self.target_ros.publish_position_target(*thold)  # fly target to start
            stt = self.target_ros.state()
            if stt.mode != 'OFFBOARD':
                self.target_ros.set_mode('OFFBOARD')
            if not stt.armed:
                self.target_ros.arm(True)
            if self.use_sim_time and now is not None:
                self.ros.wait_sim_time(now + 0.1, wall_timeout=2.0)
            else:
                time.sleep(0.1)

    # --- reset -----------------------------------------------------------------
    def reset(self, *, seed=None, options=None):
        rng = np.random.default_rng(seed) if seed is not None else self._rng
        jitter = rng.uniform(-self.cfg['target_jitter'],
                             self.cfg['target_jitter'], 3)
        jitter[2] = 0.0
        start = self._tspawn + jitter
        start[2] = self.cfg['target_altitude']
        self._target_start = start
        self._target_policy = make_target_policy(
            self.cfg['target_policy'], list(start),
            **(self.cfg.get('target_policy_params') or {}))

        # interceptor recovery first (target uncommanded here), then co-settle
        # BOTH to their starts so they are airborne when the episode begins
        obs, info = super().reset(seed=seed, options=options)
        self._cosettle(self._spawn, start)
        # restart the episode clock after ALL recovery so t=0 is the true start
        self._t0 = self.ros.sim_time() if self.use_sim_time else None
        self._t = 0.0
        self._step_i = 0
        state = self._state()
        self._prev_state = state
        return self._build_obs(state), info

    def close(self):
        try:
            self.target_ros.shutdown()
        except Exception:
            pass
        super().close()
