"""Base Gymnasium environment for drone-to-drone interception (PX4 in the loop).

Wraps the ROS2/PX4/Gazebo loop. ``reset`` repositions both drones and
re-establishes the interceptor's armed/OFFBOARD state; ``step`` applies the
action through the chosen setpoint interface, lets the sim advance one control
tick, then reads fresh state from the continuously-spinning ROS node to build
the observation, reward and termination signals.

Subclasses implement the action space and ``_apply_action`` (see
high_level_env.py and low_level_env.py). The 13-d base observation is shared.
"""
import time

import gymnasium as gym
from gymnasium import spaces
import numpy as np

from ..px4_reset import reset_land_rearm, reset_teleport
from ..ros_interface import DroneRosInterface
from ..target_policy import make_target_policy

DEFAULT_CONFIG = {
    'world': 'default',
    'interceptor_entity': 'x500_d435_0',
    'target_entity': 'x3_uav_1',
    'interceptor_ns': 'interceptor',
    'target_ns': 'target',
    'interceptor_spawn': [0.0, 0.0, 0.0],
    'target_spawn': [10.0, 0.0, 0.0],
    'control_rate_hz': 20.0,
    'max_episode_steps': 400,
    'capture_radius': 1.0,
    'arena_radius': 60.0,
    'min_altitude': 0.5,
    'max_altitude': 30.0,
    'pos_scale': 50.0,
    'vel_scale': 10.0,
    'target_altitude': 4.0,
    # reward weights
    'w_close': 1.0,
    'w_range': 0.05,
    'w_align': 0.2,
    'w_effort': 0.01,
    'r_capture': 100.0,
    'r_crash': 100.0,
    'r_oob': 50.0,
    # curriculum target
    'target_policy': 'static',
    'target_policy_params': {},
    # reset
    'use_teleport': True,
    'settle_steps': 20,
    # init randomization (half-extents around spawn)
    'init_jitter': 3.0,
}


class BaseInterceptEnv(gym.Env):
    """Shared interception env; subclasses define the action interface."""

    metadata = {'render_modes': []}

    def __init__(self, config=None, ros=None):
        super().__init__()
        self.cfg = dict(DEFAULT_CONFIG)
        if config:
            self.cfg.update(config)

        self.control_dt = 1.0 / float(self.cfg['control_rate_hz'])
        self.ros = ros or DroneRosInterface(self.cfg['interceptor_ns'],
                                            self.cfg['target_ns'])

        self._np = np.random.default_rng()
        self._step = 0
        self._episode = 0
        self._t = 0.0
        self._prev_range = None
        self._prev_action = None
        self._target_policy = None
        self._target_start = list(self.cfg['target_spawn'])

        # Observation space is shared; subclasses may extend via _extra_obs_dim.
        obs_dim = 13 + self._extra_obs_dim()
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)
        self.action_space = self._build_action_space()

    # --- hooks for subclasses ---------------------------------------------
    def _extra_obs_dim(self):
        return 0

    def _build_action_space(self):
        raise NotImplementedError

    def _apply_action(self, action):
        raise NotImplementedError

    def _extra_obs(self, interceptor_odom):
        return np.zeros((0,), dtype=np.float32)

    # --- state helpers -----------------------------------------------------
    def _world_position(self, odom, spawn):
        p = odom.pose.pose.position
        return np.array([p.x + spawn[0], p.y + spawn[1], p.z + spawn[2]])

    def _velocity(self, odom):
        v = odom.twist.twist.linear
        return np.array([v.x, v.y, v.z])

    def _states(self):
        """Return (interceptor_pos, interceptor_vel, target_pos, target_vel)."""
        io = self.ros.interceptor_odom()
        to = self.ros.target_odom()
        if io is None:
            return None
        ipos = self._world_position(io, self.cfg['interceptor_spawn'])
        ivel = self._velocity(io)
        if to is not None:
            tpos = self._world_position(to, self.cfg['target_spawn'])
            tvel = self._velocity(to)
        else:
            tpos = np.array(self._target_start)
            tvel = np.zeros(3)
        return ipos, ivel, tpos, tvel

    def _build_obs(self):
        st = self._states()
        if st is None:
            return np.zeros(self.observation_space.shape, dtype=np.float32)
        ipos, ivel, tpos, tvel = st
        rel_pos = tpos - ipos
        rel_vel = tvel - ivel
        rng = float(np.linalg.norm(rel_pos))
        los = rel_pos / rng if rng > 1e-6 else np.zeros(3)
        ps, vs = self.cfg['pos_scale'], self.cfg['vel_scale']
        base = np.concatenate([
            rel_pos / ps, rel_vel / vs, ivel / vs, [rng / ps], los,
        ]).astype(np.float32)
        extra = self._extra_obs(self.ros.interceptor_odom())
        return np.concatenate([base, extra]).astype(np.float32)

    def _range(self):
        st = self._states()
        if st is None:
            return float('inf')
        ipos, _, tpos, _ = st
        return float(np.linalg.norm(tpos - ipos))

    # --- target drive ------------------------------------------------------
    def _drive_target(self):
        if self._target_policy is None:
            return
        x, y, z = self._target_policy.setpoint(self._t)
        # Convert world position back to the target's odom frame for its setpoint.
        sp = self.cfg['target_spawn']
        self.ros.publish_target_position(x - sp[0], y - sp[1], z - sp[2])

    # --- gym API -----------------------------------------------------------
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self._np = np.random.default_rng(seed)

        self._step = 0
        self._t = 0.0
        self._prev_range = None
        self._prev_action = None
        self._episode += 1

        jitter = self.cfg['init_jitter']
        ispawn = np.array(self.cfg['interceptor_spawn'], dtype=float)
        tspawn = np.array(self.cfg['target_spawn'], dtype=float)
        i_xyz = ispawn + self._np.uniform(-jitter, jitter, 3)
        i_xyz[2] = self.cfg['target_altitude']
        t_xyz = tspawn + self._np.uniform(-jitter, jitter, 3)
        t_xyz[2] = self.cfg['target_altitude']
        self._target_start = list(t_xyz)

        self._target_policy = make_target_policy(
            self.cfg['target_policy'], self._target_start,
            **self.cfg.get('target_policy_params', {}))

        if self.cfg['use_teleport']:
            reset_teleport(self.ros, self.cfg['world'],
                           self.cfg['interceptor_entity'], self.cfg['target_entity'],
                           tuple(i_xyz), tuple(t_xyz),
                           settle_steps=self.cfg['settle_steps'], dt=self.control_dt)
        else:
            reset_land_rearm(self.ros, tuple(i_xyz),
                             settle_steps=self.cfg['settle_steps'], dt=self.control_dt)

        obs = self._build_obs()
        self._prev_range = self._range()
        return obs, {}

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        self._apply_action(action)
        self._drive_target()

        time.sleep(self.control_dt)   # let the sim advance one control tick
        self._t += self.control_dt
        self._step += 1

        obs = self._build_obs()
        reward, terminated, truncated, info = self._evaluate(action)
        self._prev_action = action
        return obs, reward, terminated, truncated, info

    def _evaluate(self, action):
        st = self._states()
        info = {}
        if st is None:
            return 0.0, False, False, info
        ipos, ivel, tpos, _ = st
        rel = tpos - ipos
        rng = float(np.linalg.norm(rel))

        reward = 0.0
        if self._prev_range is not None:
            reward += self.cfg['w_close'] * (self._prev_range - rng)
        reward -= self.cfg['w_range'] * rng
        speed = float(np.linalg.norm(ivel))
        if speed > 1e-3 and rng > 1e-3:
            cos_align = float(np.dot(ivel, rel) / (speed * rng))
            reward += self.cfg['w_align'] * cos_align
        reward -= self.cfg['w_effort'] * float(np.dot(action, action))
        self._prev_range = rng

        terminated = False
        truncated = False
        alt = ipos[2]
        horiz = float(np.linalg.norm(ipos[:2] - np.array(self.cfg['interceptor_spawn'])[:2]))
        if rng <= self.cfg['capture_radius']:
            reward += self.cfg['r_capture']
            terminated = True
            info['outcome'] = 'capture'
        elif alt < self.cfg['min_altitude']:
            reward -= self.cfg['r_crash']
            terminated = True
            info['outcome'] = 'crash'
        elif horiz > self.cfg['arena_radius'] or alt > self.cfg['max_altitude']:
            reward -= self.cfg['r_oob']
            terminated = True
            info['outcome'] = 'out_of_bounds'
        elif self._step >= self.cfg['max_episode_steps']:
            truncated = True
            info['outcome'] = 'timeout'

        info['range'] = rng
        return reward, terminated, truncated, info

    def close(self):
        try:
            self.ros.shutdown()
        except Exception:
            pass
