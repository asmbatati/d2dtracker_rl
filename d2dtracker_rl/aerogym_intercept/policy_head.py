"""Fly a trained interception policy in the full two-drone sim (RL head).

The orchestration head for ``drone_interception_sim``'s
``pipeline.launch.py orchestrator:=rl``: it runs an SB3 policy trained in the
AeroGym ``InterceptEnv`` on the interceptor of the benchmark scene, so the
learned controller is measured by the same ``interception_metrics`` logger as
the PN baseline and the pipeline backends.

The observation is rebuilt as in training: AeroGym's base block (position /
``pos_scale``, velocity / ``vel_scale``, quaternion, body rates; raw
``local_position/odom`` fields, interceptor frame = world frame) plus the
``intercept`` task block computed by :class:`InterceptTask` itself, with the
target in the common world frame via its spawn offset.

Velocity frames: MAVROS publishes the odom twist in the BODY frame. Training
used it as-is, so the interceptor velocity stays body-frame here too. The
training target held yaw 0, where body = world, so its velocity is rotated to
world here: the benchmark target yaws along its path, and its raw body twist
would always read "flying forward along +x". The action is mapped
like AeroGym's ``velocity`` interface: ENU velocity ``a * v_max``, yaw along
the velocity, ``setpoint_raw/local`` at the training control period
(``control_dt``, sim time).

Phases: Wait (both odoms + FCU) -> Climb (arm, OFFBOARD, hold the current xy
at ``start_alt``, the altitude the policy was trained from) -> Pursue (policy)
-> Return (AUTO.RTL once the true range is inside ``capture_radius``).
"""
import math

from d2dtracker_interception.frames import body_to_world
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class PolicyHead(Node):
    """Arm/OFFBOARD sequencing + SB3 policy inference for the interceptor."""

    def __init__(self):
        super().__init__('policy_head')
        self.declare_parameter('model', '')
        self.declare_parameter('v_max', 8.0)
        self.declare_parameter('control_dt', 0.05)
        self.declare_parameter('pos_scale', 30.0)
        self.declare_parameter('vel_scale', 10.0)
        self.declare_parameter('start_alt', 5.0)
        self.declare_parameter('capture_radius', 1.0)
        self.declare_parameter('target_spawn', [10.0, 0.0, 0.0])
        self.declare_parameter('deterministic', True)

        g = self.get_parameter
        self.v_max = float(g('v_max').value)
        self.ps = float(g('pos_scale').value)
        self.vs = float(g('vel_scale').value)
        self.start_alt = float(g('start_alt').value)
        self.capture_radius = float(g('capture_radius').value)
        self.tspawn = np.asarray(g('target_spawn').value, dtype=float)
        self.deterministic = bool(g('deterministic').value)

        from aerogym_px4.tasks.base import TASKS
        from . import task  # noqa: F401  (registers 'intercept')
        self.task = TASKS['intercept']({'capture_radius': self.capture_radius})
        self.model = self._load(g('model').value)

        self.odom = None
        self.todom = None
        self.state = State()
        self.phase = 'Wait'
        self.hold = None
        self.t0 = None
        self._ticks = 0

        self.create_subscription(Odometry, 'interceptor_odom',
                                 lambda m: setattr(self, 'odom', m),
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, 'target_odom',
                                 lambda m: setattr(self, 'todom', m),
                                 qos_profile_sensor_data)
        self.create_subscription(State, 'mavros/state',
                                 lambda m: setattr(self, 'state', m), 10)
        self.sp_pub = self.create_publisher(PositionTarget,
                                            'mavros/setpoint_raw/local', 10)
        self.phase_pub = self.create_publisher(String, 'mission/phase', 10)
        self.arm_cli = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.mode_cli = self.create_client(SetMode, 'mavros/set_mode')
        self.create_timer(float(g('control_dt').value), self._tick)
        self.create_timer(1.0, self._sequence)
        self.get_logger().info(f'policy head up (model={g("model").value})')

    def _load(self, path):
        from stable_baselines3 import PPO, SAC
        for cls in (SAC, PPO):
            try:
                return cls.load(path, device='cpu')
            except Exception:
                continue
        raise RuntimeError(f'could not load policy {path!r}')

    # --- state ---------------------------------------------------------------
    def _world(self):
        p, v = self.odom.pose.pose.position, self.odom.twist.twist.linear
        q, w = self.odom.pose.pose.orientation, self.odom.twist.twist.angular
        tp, to = self.todom.pose.pose.position, self.todom.pose.pose.orientation
        tv = self.todom.twist.twist.linear
        return {'position': np.array([p.x, p.y, p.z]),
                'velocity': np.array([v.x, v.y, v.z]),
                'quaternion': np.array([q.x, q.y, q.z, q.w]),
                'omega': np.array([w.x, w.y, w.z]),
                'target_position': np.array([tp.x, tp.y, tp.z]) + self.tspawn,
                'target_velocity': body_to_world([to.x, to.y, to.z, to.w],
                                                 [tv.x, tv.y, tv.z])}

    def _obs(self, st, t):
        base = np.concatenate([st['position'] / self.ps, st['velocity'] / self.vs,
                               st['quaternion'], st['omega']])
        obs = np.concatenate([base, self.task.task_obs(st, t)]).astype(np.float32)
        return np.clip(np.nan_to_num(obs, nan=0.0, posinf=100.0, neginf=-100.0),
                       -100.0, 100.0)

    def _range(self, st):
        return float(np.linalg.norm(st['target_position'] - st['position']))

    # --- setpoints -----------------------------------------------------------
    def _publish(self, vel=None, pos=None, yaw=0.0):
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        ign = (PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY
               + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW_RATE)
        if vel is not None:
            sp.type_mask = (ign + PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY
                            + PositionTarget.IGNORE_PZ)
            sp.velocity.x, sp.velocity.y, sp.velocity.z = (float(c) for c in vel)
        else:
            sp.type_mask = (ign + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY
                            + PositionTarget.IGNORE_VZ)
            sp.position.x, sp.position.y, sp.position.z = (float(c) for c in pos)
        sp.yaw = float(yaw)
        self.sp_pub.publish(sp)

    # --- loops -----------------------------------------------------------------
    def _tick(self):
        """Control-period loop: setpoint stream (Climb) / policy (Pursue)."""
        self.phase_pub.publish(String(data=self.phase))
        if self.odom is None or self.todom is None:
            return
        st = self._world()
        if self.phase in ('Wait', 'Climb'):
            if self.hold is None:
                self.hold = np.array([st['position'][0], st['position'][1],
                                      self.start_alt])
            self._publish(pos=self.hold)
            if (self.phase == 'Climb' and self.state.armed
                    and self.state.mode == 'OFFBOARD'
                    and st['position'][2] >= self.start_alt - 0.3):
                self._to('Pursue')
                self.t0 = self.get_clock().now().nanoseconds * 1e-9
        elif self.phase == 'Pursue':
            if self._range(st) <= self.capture_radius:
                self.get_logger().info(f'CAPTURE (range={self._range(st):.2f} m)')
                self._to('Return')
                return
            t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
            action, _ = self.model.predict(self._obs(st, t),
                                           deterministic=self.deterministic)
            v = np.clip(np.asarray(action, float)[:3], -1.0, 1.0) * self.v_max
            self._publish(vel=v, yaw=math.atan2(v[1], v[0]))

    def _sequence(self):
        """1 Hz: arm / mode requests for the current phase."""
        if not self.state.connected or self.odom is None or self.todom is None:
            return
        if self.phase == 'Wait':
            self._to('Climb')
        elif self.phase == 'Climb':
            if self.state.mode != 'OFFBOARD':
                self._mode('OFFBOARD')
            if not self.state.armed and self.arm_cli.service_is_ready():
                self.arm_cli.call_async(CommandBool.Request(value=True))
        elif self.phase == 'Return' and self.state.mode != 'AUTO.RTL':
            self._mode('AUTO.RTL')

    def _mode(self, mode):
        if self.mode_cli.service_is_ready():
            self.mode_cli.call_async(SetMode.Request(custom_mode=mode))

    def _to(self, phase):
        self.get_logger().info(f'phase: {self.phase} -> {phase}')
        self.phase = phase


def main(args=None):
    rclpy.init(args=args)
    node = PolicyHead()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
