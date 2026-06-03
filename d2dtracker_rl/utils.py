"""Small math helpers for the interception RL environments (no ROS imports)."""
import math

import numpy as np


def quat_to_euler(x, y, z, w):
    """Quaternion -> (roll, pitch, yaw) in radians (ZYX convention)."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def clip_vector(vec, max_norm):
    """Scale ``vec`` so its L2 norm does not exceed ``max_norm``."""
    n = float(np.linalg.norm(vec))
    if n > max_norm and n > 0.0:
        return vec * (max_norm / n)
    return vec
