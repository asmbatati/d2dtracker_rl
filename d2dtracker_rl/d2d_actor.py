import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np

class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, max_action):
        super(Actor, self).__init__()
        self.layer_1 = nn.Linear(state_dim, 400)
        self.layer_2 = nn.Linear(400, 300)
        self.layer_3 = nn.Linear(300, action_dim)
        self.max_action = max_action

    def forward(self, x):
        x = torch.relu(self.layer_1(x))
        x = torch.relu(self.layer_2(x))
        return self.max_action * torch.tanh(self.layer_3(x))

class Critic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Critic, self).__init__()
        self.layer_1 = nn.Linear(state_dim + action_dim, 400)
        self.layer_2 = nn.Linear(400, 300)
        self.layer_3 = nn.Linear(300, 1)

    def forward(self, x, u):
        x = torch.cat([x, u], 1)
        x = torch.relu(self.layer_1(x))
        x = torch.relu(self.layer_2(x))
        return self.layer_3(x)

class TD3:
    def __init__(self, state_dim, action_dim, max_action):
        self.actor = Actor(state_dim, action_dim, max_action).to("cuda")
        self.actor_target = Actor(state_dim, action_dim, max_action).to("cuda")
        self.actor_optimizer = optim.Adam(self.actor.parameters())
        
        self.critic = Critic(state_dim, action_dim).to("cuda")
        self.critic_target = Critic(state_dim, action_dim).to("cuda")
        self.critic_optimizer = optim.Adam(self.critic.parameters())
        
        self.max_action = max_action
        self.tau = 0.005  # Target network update rate

    def train(self, replay_buffer, batch_size, discount, tau, policy_noise, noise_clip, policy_freq):
        state, action, reward, next_state, done = replay_buffer.sample(batch_size)
        # Implement TD3 training steps
        # Details omitted for brevity

    def get_action(self, state):
        state = torch.FloatTensor(state.reshape(1, -1)).to("cuda")
        return self.actor(state).cpu().data.numpy().flatten()
