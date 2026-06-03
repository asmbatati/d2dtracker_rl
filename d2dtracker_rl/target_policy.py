"""Scripted target-drone motion policies used to drive the RL curriculum.

The target is flown via position setpoints (it is its own PX4 instance). Each
policy returns the next desired target position given the elapsed episode time
and the target's start position.
"""
import math


class TargetPolicy:
    """Base class: produce a target position setpoint over time."""

    def __init__(self, start, **kwargs):
        self.start = list(start)
        self.params = kwargs

    def setpoint(self, t):
        raise NotImplementedError


class StaticTarget(TargetPolicy):
    """Hovers in place (easiest curriculum stage)."""

    def setpoint(self, t):
        return tuple(self.start)


class ConstantVelocityTarget(TargetPolicy):
    """Moves in a straight line at a constant velocity vector."""

    def setpoint(self, t):
        v = self.params.get('velocity', (1.0, 0.0, 0.0))
        return (self.start[0] + v[0] * t,
                self.start[1] + v[1] * t,
                self.start[2] + v[2] * t)


class EvasiveTarget(TargetPolicy):
    """Sinusoidal weave around a drifting centre (hardest stage)."""

    def setpoint(self, t):
        v = self.params.get('velocity', (0.8, 0.0, 0.0))
        amp = self.params.get('amplitude', 3.0)
        omega = self.params.get('omega', 0.6)
        cx = self.start[0] + v[0] * t
        cy = self.start[1] + v[1] * t + amp * math.sin(omega * t)
        cz = self.start[2] + 0.5 * amp * math.sin(0.5 * omega * t)
        return (cx, cy, cz)


POLICIES = {
    'static': StaticTarget,
    'constant_velocity': ConstantVelocityTarget,
    'evasive': EvasiveTarget,
}


def make_target_policy(kind, start, **kwargs):
    """Factory: build a target policy by name (see ``POLICIES``)."""
    if kind not in POLICIES:
        raise ValueError(f'unknown target policy {kind!r}; choices: {list(POLICIES)}')
    return POLICIES[kind](start, **kwargs)
