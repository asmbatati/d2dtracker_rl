import rclpy
from rclpy.node import Node
import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
from gymnasium import spaces
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Empty
import subprocess
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import Quaternion, Vector3
from mavros.base import SENSOR_QOS

def reset_gazebo_world():
    """Send a soft reset command to Gazebo."""
    try:
        subprocess.run([
            'gz', 'service', '-s', '/world/default/control',
            '--reqtype', 'gz.msgs.WorldControl',
            '--reptype', 'gz.msgs.Empty',
            '--timeout', '5000',
            '--req', 'reset {all: true}'
        ], check=True)
        print("Gazebo world soft reset successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Failed to soft reset Gazebo world: {e}")

class DroneEnv(Node, gym.Env):
    def __init__(self):
        Node.__init__(self, 'drone_env')
        gym.Env.__init__(self)

        # Define action and observation space
        # Actions: thrust, roll rate, pitch rate, yaw rate
        self.action_space = spaces.Box(
            low=np.array([-10, -10, -10, 0]),  # body_rate x, y, z and thrust
            high=np.array([10, 10, 10, 1]),
            dtype=np.float32
        )

        # Observations: drone's position and goal position
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(6,), dtype=np.float32)
        
        # QoS profile for subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ROS 2 clients for arming and setting offboard mode
        self.arming_client = self.create_client(CommandBool, '/interceptor/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/interceptor/mavros/set_mode')
        
        # ROS 2 publishers and subscribers
        self.attitude_publisher = self.create_publisher(AttitudeTarget, '/interceptor/mavros/setpoint_raw/attitude', 10)
        self.pose_subscriber = self.create_subscription(PoseStamped, '/interceptor/mavros/local_position/pose', self._update_drone_position, qos_profile_sensor_data)
        
        self.drone_position = np.zeros(3)
        self.goal_position = np.random.uniform(-10, 10, size=3)

        # Ensure services are available
        self._wait_for_services()

    def _wait_for_services(self):
        """Ensure arming and set_mode services are available."""
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        self.get_logger().info('Arming service is ready.')
        
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')
        self.get_logger().info('Set_mode service is ready.')

    def _update_drone_position(self, msg):
        """Update the current drone position from MAVROS pose topic."""
        self.drone_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    
    def _arm_and_set_offboard(self):
        """Arm the drone and set the mode to offboard."""
        # Set mode to AUTO.LOITER before arming
        loiter_request = SetMode.Request()
        loiter_request.custom_mode = "AUTO.LOITER"
        self.set_mode_client.call_async(loiter_request)
        self.get_logger().info('Set mode to AUTO.LOITER')
        time.sleep(2)  # Give it some time to switch modes

        # Arm the drone
        arm_request = CommandBool.Request()
        arm_request.value = True
        arm_future = self.arming_client.call_async(arm_request)

        # Wait for arming confirmation
        rclpy.spin_until_future_complete(self, arm_future)
        if arm_future.result().success:
            self.get_logger().info('Drone armed successfully')
        else:
            self.get_logger().error('Failed to arm the drone')

        # Set mode to OFFBOARD
        offboard_request = SetMode.Request()
        offboard_request.custom_mode = "OFFBOARD"
        offboard_future = self.set_mode_client.call_async(offboard_request)

        # Wait for OFFBOARD confirmation
        rclpy.spin_until_future_complete(self, offboard_future)
        if offboard_future.result().mode_sent:
            self.get_logger().info('Drone set to OFFBOARD mode successfully')
        else:
            self.get_logger().error('Failed to set OFFBOARD mode')

    def reset(self, **kwargs):
        """Reset the environment for a new episode."""
        # Reset Gazebo simulation
        reset_gazebo_world()
        
        # Arm and set to offboard mode
        self._arm_and_set_offboard()

        # Randomize goal position
        self.goal_position = np.random.uniform(-10, 10, size=3)
        
        # Return initial observation and an empty info dictionary
        obs = np.concatenate([self.drone_position, self.goal_position])
        return obs, {}

    def step(self, action):
        """Apply action and compute the next state, reward, terminated, truncated, and info."""
        # Extract the action components
        body_rate = action[:3]
        thrust = 300*action[3]

        # Create and publish AttitudeTarget message
        attitude_target = AttitudeTarget()
        attitude_target.type_mask = AttitudeTarget.IGNORE_ATTITUDE  # Ignore orientation
        attitude_target.body_rate = Vector3(
            x=float(body_rate[0]),
            y=float(body_rate[1]),
            z=float(body_rate[2])
        )
        attitude_target.thrust = float(thrust)
        self.attitude_publisher.publish(attitude_target)

        # Log the published AttitudeTarget message
        self.get_logger().info(
            f"Published AttitudeTarget:\n"
            f"  Body rates: roll_rate={attitude_target.body_rate.x:.3f}, "
            f"pitch_rate={attitude_target.body_rate.y:.3f}, yaw_rate={attitude_target.body_rate.z:.3f}\n"
            f"  Thrust: {attitude_target.thrust:.3f}"
            )

        # Calculate reward based on distance to goal
        distance_to_goal = np.linalg.norm(self.drone_position - self.goal_position)
        reward = -distance_to_goal  # Negative reward for distance to encourage getting closer

        # Check if the episode is done
        terminated = distance_to_goal < 1.0  # Success threshold

        # Truncated is always False in this example
        truncated = False

        # Get the current observation
        obs = np.concatenate([self.drone_position, self.goal_position])
        return obs, reward, terminated, truncated, {}

    def close(self):
        """Shutdown the ROS node."""
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    # Create the environment
    env = DroneEnv()
    
    # Define PPO model
    model = PPO('MlpPolicy', env, verbose=1)
    
    # Training loop
    try:
        model.learn(total_timesteps=10000)
        model.save("ppo_drone_model")
    finally:
        env.close()

if __name__ == '__main__':
    main()
