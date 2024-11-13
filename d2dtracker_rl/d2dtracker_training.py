import rclpy
from rclpy.node import Node
import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
from gymnasium import spaces
from mavros_msgs.msg import OverrideRCIn
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
import subprocess
import time

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
        self.action_space = spaces.Box(low=np.array([-1, -1, -1, -1]), high=np.array([1, 1, 1, 1]), dtype=np.float32)
        
        # Observations: drone's position and goal position
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(6,), dtype=np.float32)
        
        # ROS 2 publishers and subscribers
        self.goal_position = np.random.uniform(-10, 10, size=3)
        self.pose_subscriber = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._update_drone_position, 10)
        self.rc_publisher = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.drone_position = np.zeros(3)

    def _update_drone_position(self, msg):
        """Update the current drone position from MAVROS pose topic."""
        self.drone_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    
    def reset(self, **kwargs):
        """Reset the environment for a new episode."""
        # Reset Gazebo Garden simulation
        reset_gazebo_world()
        
        # Randomize goal position
        self.goal_position = np.random.uniform(-10, 10, size=3)
        
        # Return initial observation and an empty info dictionary
        obs = np.concatenate([self.drone_position, self.goal_position])
        return obs, {}  # Returning the observation and an empty info dictionary

    
    def step(self, action):
        """Apply action and compute the next state, reward, terminated, truncated, and info."""
        thrust, roll_rate, pitch_rate, yaw_rate = action
        
        # Send the command through MAVROS (simplified for example purposes)
        rc_msg = OverrideRCIn()
        rc_msg.channels[2] = int((thrust + 1) * 1000)  # Example conversion, adjust as needed
        rc_msg.channels[0] = int((roll_rate + 1) * 1000)
        rc_msg.channels[1] = int((pitch_rate + 1) * 1000)
        rc_msg.channels[3] = int((yaw_rate + 1) * 1000)
        self.rc_publisher.publish(rc_msg)
        
        # Calculate the reward based on the distance to the goal
        distance_to_goal = np.linalg.norm(self.drone_position - self.goal_position)
        reward = -distance_to_goal  # Negative reward for distance to encourage getting closer
        
        # Check if the episode is done
        terminated = distance_to_goal < 1.0  # Success threshold
        
        # Set truncated to False (no time-based truncation in this example)
        truncated = False

        # Get observation
        obs = np.concatenate([self.drone_position, self.goal_position])
        
        # Return observation, reward, terminated, truncated, and info dictionary
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
