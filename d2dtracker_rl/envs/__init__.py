"""Gymnasium env registration for the interception environments."""
from gymnasium.envs.registration import register

from .base_intercept_env import BaseInterceptEnv
from .high_level_env import HighLevelInterceptEnv
from .low_level_env import LowLevelInterceptEnv

register(
    id='D2DIntercept-HighLevel-v0',
    entry_point='d2dtracker_rl.envs.high_level_env:HighLevelInterceptEnv',
    max_episode_steps=400,
)

register(
    id='D2DIntercept-LowLevel-v0',
    entry_point='d2dtracker_rl.envs.low_level_env:LowLevelInterceptEnv',
    max_episode_steps=400,
)

__all__ = ['BaseInterceptEnv', 'HighLevelInterceptEnv', 'LowLevelInterceptEnv']
