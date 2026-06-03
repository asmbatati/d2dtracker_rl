"""High-level interception env (curriculum stage 1).

The agent outputs an ENU velocity command; PX4 handles stabilization. This is
the easiest-to-learn, most stable interface and the primary working env.
"""
import math

from gymnasium import spaces
import numpy as np

from .base_intercept_env import BaseInterceptEnv


class HighLevelInterceptEnv(BaseInterceptEnv):
    """Action = normalized ENU velocity setpoint in [-1, 1]^3 (scaled to v_max)."""

    def __init__(self, config=None, ros=None):
        self.v_max = (config or {}).get('v_max', 8.0)
        super().__init__(config=config, ros=ros)

    def _build_action_space(self):
        return spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)

    def _apply_action(self, action):
        vx, vy, vz = (float(np.clip(a, -1.0, 1.0)) * self.v_max for a in action[:3])
        yaw = math.atan2(vy, vx)
        self.ros.publish_velocity_target(vx, vy, vz, yaw=yaw)
