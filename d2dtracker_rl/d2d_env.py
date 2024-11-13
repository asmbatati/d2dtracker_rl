import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty
from squaternion import Quaternion

GOAL_REACHED_DIST = 0.3
COLLISION_DIST = 0.35
TIME_DELTA = 0.2

class GazeboEnv(Node):
    def __init__(self):
        super().__init__('gazebo_env')
        self.goal_x = 1.0
        self.goal_y = 0.0
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.set_state = self.create_publisher(ModelState, "gazebo/set_model_state", 10)
        self.unpause = self.create_client(Empty, "/unpause_physics")
        self.pause = self.create_client(Empty, "/pause_physics")
        self.reset_proxy = self.create_client(Empty, "/reset_world")
        self.req = Empty.Request()
        self.state_dim = 24  # Adjust based on specific environment setup
        self.action_dim = 2   # Linear and angular velocities

    def reset(self):
        # Reset environment
        self.call_service(self.reset_proxy)
        return np.zeros(self.state_dim)  # Placeholder state after reset

    def step(self, action):
        vel_cmd = Twist()
        vel_cmd.linear.x = float(action[0])
        vel_cmd.angular.z = float(action[1])
        self.vel_pub.publish(vel_cmd)
        time.sleep(TIME_DELTA)
        done, reward = self.check_goal()
        next_state = self.get_observation()
        return next_state, reward, done, {}

    def get_observation(self):
        return np.random.rand(self.state_dim)  # Placeholder observation

    def check_goal(self):
        # Define goal checking and collision logic
        # Return done status and reward
        done = False
        reward = 0.0
        # Example reward setup
        return done, reward

    def call_service(self, client):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')
        try:
            client.call_async(Empty.Request())
        except Exception as e:
            self.get_logger().info(f"Service call failed: {e}")
