#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import torch
from d2dtracker_rl.d2d_env import GazeboEnv
from d2dtracker_rl.d2d_actor import TD3

class TestD2DNode(Node):
    def __init__(self):
        super().__init__('test_d2d_node')
        
        # Define parameters and load model
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.state_dim = 24  # Example state dimension
        self.action_dim = 2   # Example action dimension
        self.max_action = 1.0
        self.model_dir = './models/'  # Adjust path as needed
        self.model_name = 'td3_interceptor_actor.pth'

        # Initialize environment and network
        self.env = GazeboEnv()
        self.network = TD3(self.state_dim, self.action_dim, self.max_action)
        
        # Load the trained actor model
        self.load_model()

        # Testing variables
        self.state = self.env.reset()
        self.done = False
        self.episode_timesteps = 0
        self.max_timesteps = 500

    def load_model(self):
        actor_path = f"{self.model_dir}{self.model_name}"
        try:
            self.network.actor.load_state_dict(torch.load(actor_path, map_location=self.device))
            self.get_logger().info("Successfully loaded model for testing.")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")

    def test_step(self):
        if self.done:
            self.state = self.env.reset()
            self.done = False
            self.episode_timesteps = 0

        # Get the action from the network
        action = self.network.get_action(np.array(self.state))
        a_in = [(action[0] + 1) / 2, action[1]]
        
        # Step in the environment
        next_state, reward, self.done, _ = self.env.step(a_in)
        self.state = next_state
        self.episode_timesteps += 1

        if self.episode_timesteps >= self.max_timesteps:
            self.done = True
            self.get_logger().info("Episode finished.")

def main(args=None):
    rclpy.init(args=args)
    test_node = TestD2DNode()
    rate = test_node.create_rate(5)  # Testing at 5 Hz
    
    try:
        while rclpy.ok():
            test_node.test_step()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
