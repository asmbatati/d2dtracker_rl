import numpy as np
import random

class ReplayBuffer:
    def __init__(self, max_size=1e6, seed=0):
        self.storage = []
        self.max_size = int(max_size)
        random.seed(seed)

    def add(self, state, action, reward, done, next_state):
        data = (state, action, reward, done, next_state)
        if len(self.storage) >= self.max_size:
            self.storage.pop(0)
        self.storage.append(data)

    def sample(self, batch_size):
        batch = random.sample(self.storage, batch_size)
        state, action, reward, done, next_state = map(np.stack, zip(*batch))
        return (
            np.array(state),
            np.array(action),
            np.array(reward).reshape(-1, 1),
            np.array(done).reshape(-1, 1),
            np.array(next_state),
        )
