"""ROS 2 interface for the interception RL environments.

A single rclpy node, spun by a background daemon thread, caches the latest
interceptor/target state and exposes thread-safe getters plus actuator helpers
(arming, mode switching, position / attitude setpoints). The Gymnasium env reads
the caches in ``step``/``reset`` instead of spinning inline - this is the core
fix over the old code, which never refreshed state inside ``step``.
"""
import threading

from mavros_msgs.msg import AttitudeTarget, PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)


class DroneRosInterface(Node):
    """Shared ROS node for the interceptor (and a scripted target)."""

    def __init__(self, interceptor_ns='interceptor', target_ns='target'):
        super().__init__('d2d_rl_interface')
        self._lock = threading.Lock()

        self._interceptor_odom = None
        self._interceptor_state = State()
        self._target_odom = None

        state_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE,
                               history=HistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(
            Odometry, f'/{interceptor_ns}/mavros/local_position/odom',
            self._int_odom_cb, qos_profile_sensor_data)
        self.create_subscription(
            State, f'/{interceptor_ns}/mavros/state',
            self._int_state_cb, state_qos)
        self.create_subscription(
            Odometry, f'/{target_ns}/mavros/local_position/odom',
            self._tgt_odom_cb, qos_profile_sensor_data)

        self.pos_pub = self.create_publisher(
            PositionTarget, f'/{interceptor_ns}/mavros/setpoint_raw/local',
            qos_profile_sensor_data)
        self.att_pub = self.create_publisher(
            AttitudeTarget, f'/{interceptor_ns}/mavros/setpoint_raw/attitude',
            qos_profile_sensor_data)
        self.tgt_pos_pub = self.create_publisher(
            PositionTarget, f'/{target_ns}/mavros/setpoint_raw/local',
            qos_profile_sensor_data)

        self.arm_client = self.create_client(
            CommandBool, f'/{interceptor_ns}/mavros/cmd/arming')
        self.mode_client = self.create_client(
            SetMode, f'/{interceptor_ns}/mavros/set_mode')
        self.tgt_arm_client = self.create_client(
            CommandBool, f'/{target_ns}/mavros/cmd/arming')
        self.tgt_mode_client = self.create_client(
            SetMode, f'/{target_ns}/mavros/set_mode')

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    # --- callbacks ---------------------------------------------------------
    def _int_odom_cb(self, msg):
        with self._lock:
            self._interceptor_odom = msg

    def _int_state_cb(self, msg):
        with self._lock:
            self._interceptor_state = msg

    def _tgt_odom_cb(self, msg):
        with self._lock:
            self._target_odom = msg

    # --- thread-safe getters ----------------------------------------------
    def interceptor_odom(self):
        with self._lock:
            return self._interceptor_odom

    def target_odom(self):
        with self._lock:
            return self._target_odom

    def interceptor_state(self):
        with self._lock:
            return self._interceptor_state

    def connected(self):
        return self.interceptor_state().connected

    # --- actuators ---------------------------------------------------------
    def publish_position_target(self, x, y, z, yaw=0.0):
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY +
                        PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX +
                        PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                        PositionTarget.IGNORE_YAW_RATE)
        sp.position.x, sp.position.y, sp.position.z = float(x), float(y), float(z)
        sp.yaw = float(yaw)
        self.pos_pub.publish(sp)

    def publish_velocity_target(self, vx, vy, vz, yaw=0.0):
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY +
                        PositionTarget.IGNORE_PZ + PositionTarget.IGNORE_AFX +
                        PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                        PositionTarget.IGNORE_YAW_RATE)
        sp.velocity.x, sp.velocity.y, sp.velocity.z = float(vx), float(vy), float(vz)
        sp.yaw = float(yaw)
        self.pos_pub.publish(sp)

    def publish_attitude_rate(self, roll_rate, pitch_rate, yaw_rate, thrust):
        sp = AttitudeTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.type_mask = AttitudeTarget.IGNORE_ATTITUDE
        sp.body_rate.x = float(roll_rate)
        sp.body_rate.y = float(pitch_rate)
        sp.body_rate.z = float(yaw_rate)
        sp.thrust = float(max(0.0, min(1.0, thrust)))
        self.att_pub.publish(sp)

    def publish_target_position(self, x, y, z, yaw=0.0):
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY +
                        PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX +
                        PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                        PositionTarget.IGNORE_YAW_RATE)
        sp.position.x, sp.position.y, sp.position.z = float(x), float(y), float(z)
        sp.yaw = float(yaw)
        self.tgt_pos_pub.publish(sp)

    def arm(self, value=True, target=False):
        client = self.tgt_arm_client if target else self.arm_client
        if client.service_is_ready():
            req = CommandBool.Request()
            req.value = value
            client.call_async(req)

    def set_mode(self, custom_mode, target=False):
        client = self.tgt_mode_client if target else self.mode_client
        if client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = custom_mode
            client.call_async(req)

    def shutdown(self):
        self._executor.shutdown()
        self.destroy_node()
