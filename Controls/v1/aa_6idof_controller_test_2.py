#!/usr/bin/env python
#!/usr/bin/env python
import time
import sys
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.animation import FuncAnimation

# Controller und Hilfsfunktionen
from single_agent_controller.controllers.low_level_ctrl_2 import LLC2
from single_agent_controller.controllers_utils.helpers import *

# Simulator-Module
from single_agent_simulator.x.system_dynamics import *
from single_agent_simulator.x.ode_solver import *
from single_agent_simulator.y.helpers import *
from single_agent_simulator.y.six_dof_model import *

# Visualiser
from single_agent_visualiser.vector_visualiser import *
from single_agent_visualiser.log_visualiser import *
from single_agent_visualiser.new_visualiser import Live3DVisualizer

# Utilities
from utils.waypoints import *
from utils import CONSTANTS
from utils.constants2 import *


def plot_desired_velocities(t_s, data_log):
    """
    Plottet die Desired Velocities (X, Y, Z) über die Simulationszeit.
    Es werden horizontale Linien bei +MAX_VEL und -MAX_VEL eingezeichnet.
    """
    fig, axes = plt.subplots(3, 2, figsize=(10, 8))
    labels = ["Desired X Velocity", "Desired Y Velocity", "Desired Z Velocity", "X Position", "Y Position", "Z Position"]
    

    x = np.arange(t_s-1)


    for i in range(3):
        axes[i, 0].plot(x, data_log[:,i], label=labels[i])
        axes[i, 0].set_xlabel("Time [s]")
        axes[i, 0].set_ylabel("Velocity [m/s]")
        axes[i, 0].legend()
    for i in range(3):
        axes[i, 1].plot(x, data_log[:,i+3], label=labels[i+3])
        axes[i, 1].set_xlabel("Time [s]")
        axes[i, 1].set_ylabel("Position [m]")
        axes[i, 1].legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":

    # Initialisiere den Low-Level Controller (LLC) mit den Konstanten aus CONSTANTS

    open('data_log.log', 'w').close()
    
    llc = LLC2(CONSTANTS['pid_params'], CONSTANTS['init_params'], MAX_VEL)
    # Setze den einzelnen Waypoint als globales Ziel
    llc.global_position_target = np.array([waypoints[0]['x'], waypoints[0]['y'], waypoints[0]['z']])
    llc.global_orientation_target_quat = llc.euler_zyx_to_quaternion(
        waypoints[0]['yaw'], waypoints[0]['pitch'], waypoints[0]['roll']
    )
    
    num_steps = 1000
    
    
    with open('data_log.log', 'ab') as log:
        for i in range(0, num_steps):

            pos_x = 1000.0-i
            pos_y = 1000.0-i
            pos_z = 1000.0-i         

            llc.update_global_position(pos_x, pos_y, pos_z)

            desired_x_vel, desired_y_vel, desired_z_vel, finished = llc.update_w_mode6()
            
            # Logge die Desired Velocities in das Array und in die separate Logdatei

            np.savetxt(log, np.array([desired_x_vel, desired_y_vel, desired_z_vel, pos_x, pos_y, pos_z]).reshape(1, -1), delimiter=',')
            
            if finished:
                print("alle waypoints finished")
                break
            
            llc.update_global_position(pos_x, pos_y, pos_z)
            
    data = np.loadtxt('data_log.log',delimiter=',')
    print(data)
    #Plotte die Desired Velocities
    plot_desired_velocities(num_steps, data)