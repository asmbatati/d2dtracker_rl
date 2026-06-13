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
import numpy as np

from aerogym_px4.core.base_env import BasePX4Env
from aerogym_px4.core.px4_reset import ensure_armed_offboard, gz_set_entity_pose
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

    # --- reset -----------------------------------------------------------------
    def reset(self, *, seed=None, options=None):
        # target first: fresh start point + policy, teleport + hold
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

        # teleport only when airborne (same EKF-safety rule as the interceptor);
        # a grounded target instead climbs to its start under the held setpoint
        # + the policy setpoints during the first episode seconds
        todom = self.target_ros.odom()
        airborne = (todom is not None and todom.pose.pose.position.z > 1.0
                    and self.target_ros.state().armed)
        if airborne:
            gz_set_entity_pose(
                self.cfg['world'], self.cfg['target_entity'], *start,
                extra_env=self.sim.child_env if self.sim is not None else None)
        hold = start - self._tspawn          # target's own odom frame
        ensure_armed_offboard(self.target_ros, tuple(hold),
                              settle_steps=10, dt=self.control_dt,
                              use_sim_time=self.use_sim_time)

        return super().reset(seed=seed, options=options)

    def close(self):
        try:
            self.target_ros.shutdown()
        except Exception:
            pass
        super().close()
