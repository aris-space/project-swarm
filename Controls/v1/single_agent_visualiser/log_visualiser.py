import numpy as np
import matplotlib.pyplot as plt

def log_visualiser(time_p, data):
    # Plot each state variable separately
    plt.figure()
    for i in range(data.shape[1]):
        plt.plot(time_p, data[:, i], label=f'State Variable {i+1}')

    plt.xlabel('Time (line index)')
    plt.ylabel('Actual State Variables')
    plt.title('Actual State over Time')
    plt.legend(['Roll Angle', 'Roll Rate', 'Pitch Angle', 'Pitch Rate', 'Yaw Angle', 'Yaw Rate', 'Depth', 'Depth Rate'])
    plt.show()