import numpy as np
from Settings import *
import matplotlib.pyplot as plt


""" Q-VALUES"""


def random_action(nr_actions=4):
    """
    Choose random action from 0 to NR_OF_PIEZOS
    :param nr_actions:  NR_OF_PIEZOS
    :return:            integer from 0 to NR_OF_PIEZOS
    """
    return np.random.randint(low=0, high=nr_actions)

def update_q_values(action, memory, q_values):

    # Filter for action
    if action in [0, 1, 2, 3]:

        # Get mean position of memory
        mean_pos = np.mean(memory, 0, dtype=int)

        # Calculate average direction of swarm movement
        avg_speed = np.mean(np.array(memory)[1:] - np.array(memory)[:-1], axis=0)

        kernel_slice_x = slice(300 - mean_pos[1], 600 - mean_pos[1])
        kernel_slice_y = slice(300 - mean_pos[0], 600 - mean_pos[0])

        # Update q values
        q_values[action] = GAMMA * q_values[action] + (1-GAMMA) * Q_VALUES_UPDATE_KERNEL[kernel_slice_x, kernel_slice_y] * avg_speed
        q_values[action] = q_values[action] / np.linalg.norm(q_values[action], axis=-1)[:, :, np.newaxis].repeat(2, -1)

        return q_values

    else:
        return q_values

def calc_action(pos0, offset, q_values=None, mode='naive'):
    """
    Calculate optimal piezo to actuate
    :param pos0:    Swarm position
    :param offset:  Offset to target
    :param mode:    Selection mode
    :return:        integer from 0 to NR_OF_PIEZOS
    """

    if np.random.rand() < EPSILON:
        return random_action()

    # Same as walk_to_pixel function
    if mode == 'naive':
        action = np.argmax(np.abs(offset))
        if not np.sign(offset[action]) == -1:
            action += 2
        return (action + 2) % 4

    # Choose action from ROI of the vector field based on the average of ROI - offset
    elif mode == 'avg':
        ROI = q_values[:, slice(max(pos0[0] - MAX_VELO, 0), max(pos0[0] + MAX_VELO, 0)),
              slice(max(pos0[1] - MAX_VELO, 0), max(pos0[1] + MAX_VELO, 0)),
              slice(0, 2)]
        action = np.argmin(np.linalg.norm(np.average(ROI, axis=(1, 2)) - offset, axis=-1))
        return (action + 2) % 4

    else:
        raise ValueError(f'Mode {mode} is unvalid')

