from single_agent_controller.controllers.low_level_ctrl_2 import LLC2
from single_agent_controller.controllers_utils.helpers import *

from single_agent_simulator.x.system_dynamics import *
from single_agent_simulator.x.ode_solver import *
from single_agent_simulator.y.helpers import *
from single_agent_simulator.y.six_dof_model import *


from single_agent_visualiser.vector_visualiser import *
from single_agent_visualiser.log_visualiser import *
from single_agent_visualiser.new_visualiser import Live3DVisualizer


from utils.waypoints import *
from utils import CONSTANTS
from utils.constants2 import *

import numpy as np
import matplotlib.pyplot as plt

import pandas as pd

from matplotlib.animation import FuncAnimation

if __name__ == "__main__":

    #enable live matplotlib plotting

    #empty log file
    open('total_state.log', 'w').close()

    #plt.ion()

    #initialize llc with initial state and store time 
    llc = LLC2(CONSTANTS['pid_params'], CONSTANTS['init_params'])
    last_update = time.time()

    #3x2 array for roll, pitch, yaw and their rates from the init_params
    angle_state = np.array([
        np.array([CONSTANTS['init_params']['roll_init'], CONSTANTS['init_params']['roll_rate_init']]),#, CONSTANTS['init_params']['roll_acc_init']]), 
        np.array([CONSTANTS['init_params']['pitch_init'], CONSTANTS['init_params']['pitch_rate_init']]),#, CONSTANTS['init_params']['pitch_acc_init']]), 
        np.array([CONSTANTS['init_params']['yaw_init'], CONSTANTS['init_params']['yaw_rate_init']])#, CONSTANTS['init_params']['yaw_acc_init']])
    ])

    position_state = np.array([
        np.array([CONSTANTS['init_params']['x_init'], CONSTANTS['init_params']['x_vel_init']]),
        np.array([CONSTANTS['init_params']['y_init'], CONSTANTS['init_params']['y_vel_init']]),
        np.array([CONSTANTS['init_params']['z_init'], CONSTANTS['init_params']['z_vel_init']])
    ])


    with open('total_state.log', 'ab') as log:
    
        vehicle_model, t_s, h_s, x = initialize()
        x[0,0] = position_state[0,0]    
        x[1,0] = position_state[1,0]
        x[2,0] = position_state[2,0]
        x[3,0] = angle_state[0,0]
        x[4,0] = angle_state[1,0]
        x[5,0] = angle_state[2,0]


        for i in range(1,3000): #planner updates

            thrustx, thrusty, thrustz, torquex, torquey, torquez = llc.update_w_mode3()

            #update torques in sim
            x[12:18,i-1] = np.array([thrustx, thrusty, thrustz, torquex, torquey, torquez])

            #rk4 => convert this into a function
            k1 = state_equations(x[:,i-1], vehicle_model)
            k2 = state_equations(x[:,i-1] + h_s*k1/2, vehicle_model)
            k3 = state_equations(x[:,i-1] + h_s*k2/2, vehicle_model)
            k4 = state_equations(x[:,i-1] + h_s*k3, vehicle_model)

            x[:,i] = x[:,i-1] + (h_s/6)*(k1+2*k2+2*k3+k4)

            #fetch position state from sim
            position_state[0,:] = np.array([x[0,i], x[6,i]])
            position_state[1,:] = np.array([x[1,i], x[7,i]])
            position_state[2,:] = np.array([x[2,i], x[8,i]])

            #fetch angle state from sim
            angle_state[0,:] = np.array([x[3,i], x[9,i]])
            angle_state[1,:] = np.array([x[4,i], x[10,i]])
            angle_state[2,:] = np.array([x[5,i], x[11,i]])

            #what needs to happen in sim: give position back in global reference frame, meaning it has to be multiplied by the orientation 

            #update angle state & position state in controller
            llc.update_global_orientation_w_state(angle_state[2,0],angle_state[1,0],angle_state[0,0])
            llc.update_global_position(position_state[0,0],position_state[1,0],position_state[2,0])

            #update angle rate state & velocity state in controller
            llc.update_actual_local_rates(angle_state[0,1],angle_state[1,1],angle_state[2,1])
            llc.update_actual_local_velocities(position_state[0,1],position_state[1,1],position_state[2,1])

            #log rate rates in a log file
            np.savetxt(log, angle_state.reshape(1, -1), delimiter=',')

            # if i == 1:
            #     fig, ax = plot_orientation(x=position_state[0,0], y=position_state[1,0], z=position_state[2,0], roll=angle_state[0,0], pitch=angle_state[1,0], yaw=angle_state[2,0])
            # else:
            #     ax.clear()
            #     fig, ax = plot_orientation(fig, ax, x=position_state[0,0], y=position_state[1,0], z=position_state[2,0], roll=angle_state[0,0], pitch=angle_state[1,0], yaw=angle_state[2,0])
            # plt.draw()  # Draw the updated plot
            # plt.pause(0.001)#(0.01-((time.time()-last_update)/1000))  # Pause to update the plot


    # Plot the data
    plot(t_s, x)