
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
import random
import gym
import matplotlib.pyplot as plt


class DQN(nn.Module):
    def __init__(self, input_shape, num_actions):
        super(DQN, self).__init__()

        self.conv1 = nn.Conv2d(input_shape[0], 32, kernel_size=8, stride=4)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=4, stride=2)
        self.conv3 = nn.Conv2d(64, 64, kernel_size=3, stride=1)

        conv_out_size = self._get_conv_out(input_shape)

        self.fc1 = nn.Linear(conv_out_size, 512)
        self.fc2 = nn.Linear(512, num_actions)

    def _get_conv_out(self, shape):
        o = self.conv1(torch.zeros(1, *shape))
        o = self.conv2(o)
        o = self.conv3(o)
        return int(np.prod(o.size()))

    def forward(self, x):
        x = x / 255.0
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = x.view(x.size(0), -1)
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        return x

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

dqn = DQN((1, 300, 300), 4).to(device)

BATCH_SIZE = 32
GAMMA = 0.99
EPS_START = 1.0
EPS_END = 0.01
EPS_DECAY = 1000000
TARGET_UPDATE = 1000
LR = 1e-3

optimizer = optim.Adam(dqn.parameters(), lr=LR)
loss_fn = nn.MSELoss()

# REPLAY BUFFER 

class ReplayBuffer(object):

    def __init__(self, capacity):
        self.capacity = capacity
        self.buffer = []
        self.position = 0

    def push(self, state, action, reward, next_state, done):
        if len(self.buffer) < self.capacity:
            self.buffer.append(None)
        self.buffer[self.position] = (state, action, reward, next_state, done)
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        state, action, reward, next_state, done = map(np.stack, zip(*batch))
        return state, action, reward, next_state, done

    def __len__(self):
        return len(self.buffer)

memory = ReplayBuffer(10000)

# EPSILON GREEDY

steps_done = 0
def select_action(state, eps_threshold):
    global steps_done
    sample = random.random()
    eps_threshold = EPS_END + (EPS_START - EPS_END) * \
        np.exp(-1. * steps_done / EPS_DECAY)
    steps_done += 1
    if sample > eps_threshold:
        with torch.no_grad():
            q_values = dqn(state)
            action = q_values.max(1)[1].view(1, 1)
    else:
        action = torch.tensor([[random.randrange(4)]], device=device, dtype=torch.long)
    return action


def train_dqn(num_episodes, env, dqn, target_dqn, optimizer, loss_fn, memory):

    episode_rewards = []
    for i_episode in range(num_episodes):
        state = env.reset()
        done = False
        total_reward = 0

    while not done:
        action = select_action(torch.tensor([state], device=device, dtype=torch.float32), EPS_END)
        next_state, reward, done, _ = env.step(action.item())
        total_reward += reward
        reward = torch.tensor([reward], device=device, dtype=torch.float32)

        memory.push(torch.tensor([state], device=device, dtype=torch.float32),
                    action,
                    reward,
                    torch.tensor([next_state], device=device, dtype=torch.float32),
                    torch.tensor([done], device=device, dtype=torch.float32))

        state = next_state

        if len(memory) >= BATCH_SIZE:

            state_batch, action_batch, reward_batch, next_state_batch, done_batch = memory.sample(BATCH_SIZE)
            q_values = dqn(state_batch).gather(1, action_batch)

            next_q_values = target_dqn(next_state_batch).detach().max(1)[0].unsqueeze(1)
            expected_q_values = reward_batch + (GAMMA * next_q_values * (1 - done_batch))

            loss = loss_fn(q_values, expected_q_values)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            if steps_done % TARGET_UPDATE == 0:
                target_dqn.load_state_dict(dqn.state_dict())

    episode_rewards.append(total_reward)

    if i_episode % 10 == 0:
        print("Episode ", i_episode, ", reward: ", total_reward)

    return episode_rewards

env = gym.make('CartPole-v0')
target_dqn = DQN((1, 300, 300), 4).to(device)
target_dqn.load_state_dict(dqn.state_dict())

episode_rewards = train_dqn(1000, env, dqn, target_dqn, optimizer, loss_fn, memory)

fig, ax = plt.subplots()
ax.plot(episode_rewards)

torch.save(dqn.state_dict(), 'dqn_cartpole.pth')

