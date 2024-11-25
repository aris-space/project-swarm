import numpy as np
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