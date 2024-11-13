#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget, State
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from d2dtracker_rl.d2d_actor import TD3
from d2dtracker_rl.d2d_nn_module import ReplayBuffer

class D2DRLRenderNode(Node):
    def __init__(self):
        super().__init__('d2d_rl_render_node')

        # Target position (0, 0, 5)
        self.goal_position = np.array([0.0, 0.0, 5.0])

        # Environment and Agent Parameters
        self.state_dim = 9  # Example observation space size
        self.action_dim = 4  # Action space [thrust, roll_rate, pitch_rate, yaw_rate]
        self.max_action = 1.0
        self.agent = TD3(self.state_dim, self.action_dim, self.max_action)
        self.replay_buffer = ReplayBuffer(buffer_size=100000, seed=0)

        # ROS 2 Publishers and Subscribers
        self.target_pub = self.create_publisher(PositionTarget, 'mavros/setpoint_raw/attitude', 10)
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'mavros/local_position/odom', self.odom_callback, 10)

        # Marker for visualizing the goal
        self.goal_marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.create_goal_marker()

        # Training parameters
        self.done = False
        self.state = None
        self.total_timesteps = 0
        self.episode_timesteps = 0
        self.episode_count = 0

    def state_callback(self, msg):
        self.armed = msg.armed

    def odom_callback(self, msg):
        # Update state with current position and orientation
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        orientation = msg.pose.pose.orientation
        distance_to_goal = np.linalg.norm(position - self.goal_position)
        self.state = np.concatenate((position, [orientation.x, orientation.y, orientation.z, distance_to_goal]))

    def create_goal_marker(self):
        # Create a marker for the goal
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal_position[0]
        marker.pose.position.y = self.goal_position[1]
        marker.pose.position.z = self.goal_position[2]
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.goal_marker_pub.publish(marker)

    def calculate_reward(self, position):
        distance_to_goal = np.linalg.norm(position - self.goal_position)
        reward = -distance_to_goal
        if distance_to_goal < 0.3:
            reward += 100
            self.done = True
        return reward

    def reset_episode(self):
        self.done = False
        self.state = None
        self.episode_timesteps = 0
        self.episode_count += 1
        self.get_logger().info(f"Starting Episode {self.episode_count}")
        self.create_goal_marker()  # Render the goal at the start of each episode

    def publish_target(self, action):
        target_msg = PositionTarget()
        target_msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        target_msg.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + \
                               PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + \
                               PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                               PositionTarget.IGNORE_YAW

        # Map action to target fields
        target_msg.thrust = np.clip(action[0], 0.0, 1.0)  # Clamp thrust to [0, 1]
        target_msg.body_roll_rate = action[1]
        target_msg.body_pitch_rate = action[2]
        target_msg.body_yaw_rate = action[3]

        self.target_pub.publish(target_msg)

    def run_episode(self):
        if self.state is None:
            return

        action = self.agent.get_action(self.state)
        self.publish_target(action)

        # Get next state and reward
        next_position = self.state[:3]  # Position data
        reward = self.calculate_reward(next_position)
        done_bool = float(self.done)

        # Store transition in replay buffer
        next_state = self.state
        self.replay_buffer.add(self.state, action, reward, done_bool, next_state)

        # Train agent
        if self.total_timesteps > 1000:
            self.agent.train(self.replay_buffer, 100)

        # Reset if episode is done
        if self.done:
            self.reset_episode()

        # Update state and timestep counts
        self.state = next_state
        self.total_timesteps += 1
        self.episode_timesteps += 1

def main(args=None):
    rclpy.init(args=args)
    node = D2DRLRenderNode()
    rate = node.create_rate(20)  # Running at 20 Hz

    try:
        while rclpy.ok():
            node.run_episode()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
