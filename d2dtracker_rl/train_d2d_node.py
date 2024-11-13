#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import torch
from d2dtracker_rl.d2d_env import GazeboEnv
from d2dtracker_rl.d2d_actor import TD3
from d2dtracker_rl.d2d_nn_module import ReplayBuffer

class TrainD2DNode(Node):
    def __init__(self):
        super().__init__('train_d2d_node')
        
        # Define parameters
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.state_dim = 24  # Example state dimension
        self.action_dim = 2   # Example action dimension
        self.max_action = 1.0
        self.buffer_size = int(1e6)
        self.batch_size = 64
        self.discount = 0.99
        self.tau = 0.005
        self.policy_noise = 0.2
        self.noise_clip = 0.5
        self.policy_freq = 2
        self.max_episodes = 1000
        self.max_timesteps = 500
        self.eval_freq = 5e3
        
        # Initialize environment, network, and replay buffer
        self.env = GazeboEnv()
        self.network = TD3(self.state_dim, self.action_dim, self.max_action)
        self.replay_buffer = ReplayBuffer(self.buffer_size, seed=0)

        # Training variables
        self.episode_num = 0
        self.done = True
        self.state = None
        self.episode_reward = 0
        self.episode_timesteps = 0
        self.total_timesteps = 0

    def train_step(self):
        if self.done:
            if self.total_timesteps != 0:
                self.network.train(
                    self.replay_buffer, self.episode_timesteps, 
                    self.batch_size, self.discount, self.tau, 
                    self.policy_noise, self.noise_clip, self.policy_freq
                )
            self.state = self.env.reset()
            self.done = False
            self.episode_timesteps = 0
            self.episode_reward = 0
            self.episode_num += 1

        action = self.network.get_action(np.array(self.state))
        action = (action + np.random.normal(0, self.policy_noise, size=self.action_dim)).clip(-self.max_action, self.max_action)
        next_state, reward, self.done, _ = self.env.step(action)
        done_bool = float(self.done) if self.episode_timesteps + 1 < self.max_timesteps else 0
        self.replay_buffer.add(self.state, action, reward, done_bool, next_state)
        
        self.state = next_state
        self.episode_timesteps += 1
        self.episode_reward += reward
        self.total_timesteps += 1

        if self.total_timesteps % self.eval_freq == 0:
            self.evaluate_model()

    def evaluate_model(self):
        self.get_logger().info(f"Evaluating model at timestep {self.total_timesteps}")
        # Add evaluation logic here if needed

def main(args=None):
    rclpy.init(args=args)
    train_node = TrainD2DNode()
    rate = train_node.create_rate(10)  # Set a rate for training steps
    
    try:
        while rclpy.ok():
            train_node.train_step()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        train_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
