import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def log_visualiser(time_p, data):
    # Plot all state variables in subplots, make all rates the lighter shade of their absolute
    variable_names = ['Roll', 'Pitch', 'Yaw', 'Depth']
    colors = ['b', 'g', 'r', 'c']

    fig, axes = plt.subplots(len(variable_names), 1, figsize=(8, 10), sharex=True)
    for i, ax in enumerate(axes):
        absolute = data[:, 2 * i]
        rate = data[:, 2 * i + 1]

        ax.plot(time_p, absolute, label=f'{variable_names[i]} Angle', color=colors[i])
        ax.plot(time_p, rate, label=f'{variable_names[i]} Rate', color=colors[i], linestyle='--', alpha=0.5)

        ax.set_ylabel('Value')
        ax.set_title(f'{variable_names[i]} over Time')
        ax.legend()

    axes[-1].set_xlabel('Time')
    plt.tight_layout()
    plt.show()


def log_visualiser2(measured_state):
    # Variable names for the absolute states
    variable_names = ['time_step', 'actual_roll', 'actual_pitch',  'actual_yaw',  'actual_depth',
                         'measured_roll', 'measured_pitch', 'measured_yaw',  'measured_depth',]
    
    # Read actual state from states_log.csv
    actual_state = pd.read_csv('states_log.csv').values
    time_indices = np.arange(actual_state.shape[0])


    _, axes = plt.subplots(len(variable_names), 1, figsize=(10, 12), sharex=True)

    for i, ax in enumerate(axes):
        ax.plot(time_indices, actual_state[:, i], label=f'Actual {variable_names[i]}')
        ax.plot(time_indices, measured_state[:, i], label=f'Measured {variable_names[i]}', linestyle='--')
        ax.set_ylabel(variable_names[i])
        ax.legend()
        ax.grid(True)

    axes[-1].set_xlabel('Time Index')
    plt.tight_layout()
    plt.show()