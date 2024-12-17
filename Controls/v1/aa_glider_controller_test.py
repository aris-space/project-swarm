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


        for i in range(1,1000): #planner updates

            # llc.convert_global_x_y_to_local_x_yaw()

            # #update angle & pos PIDs
            # llc.update_angle_pids()
            # llc.update_x_pid()
            # llc.update_z_pid()
            # #print('z', llc.desired_local_z_vel)
            # #print('x', llc.desired_local_x_vel)
            

            # #update angle rate and vel PIDs
            # torquex, torquey, torquez = llc.update_angle_rate_pids()
            # thrustx = llc.update_x_vel_pid()
            # thrustz = llc.update_z_vel_pid()
            # #print(thrustx, thrustz, torquex, torquey, torquez)

            thrustx, _, thrustz, torquex, torquey, torquez = llc.update_w_mode2()

            x[12,i-1] = thrustx
            x[13,i-1] = 0
            x[14,i-1] = thrustz
            x[15,i-1] = torquex
            x[16,i-1] = torquey
            x[17,i-1] = torquez

            #print('thrustx', thrustx)

            """write an exception for local x's => convert them first to global before integrating
            global_x = x_prev + orientation * x[0,i-1] * dt
            do rk4 with orientation * x thing

            """


            k1 = state_equations(x[:,i-1], vehicle_model)
            k2 = state_equations(x[:,i-1] + h_s*k1/2, vehicle_model)
            k3 = state_equations(x[:,i-1] + h_s*k2/2, vehicle_model)
            k4 = state_equations(x[:,i-1] + h_s*k3, vehicle_model)

            #implement integration in euler angles


            x[:,i] = x[:,i-1] + (h_s/6)*(k1+2*k2+2*k3+k4)

            #update position state in controller
            position_state[0,:] = np.array([x[0,i], x[6,i]])
            position_state[1,:] = np.array([x[1,i], x[7,i]])
            position_state[2,:] = np.array([x[2,i], x[8,i]])
            #print('x', position_state[0,0])
            #print('z', position_state[2,0])

            angle_state[0,:] = np.array([x[3,i], x[9,i]])
            angle_state[1,:] = np.array([x[4,i], x[10,i]])
            angle_state[2,:] = np.array([x[5,i], x[11,i]]) 

            #update angle state in controller
            llc.update_global_orientation_w_state(angle_state[2,0],angle_state[1,0],angle_state[0,0])
            llc.update_global_position(position_state[0,0],position_state[1,0],position_state[2,0])

            #updae angle rate state in controller
            llc.update_actual_local_rates(angle_state[0,1],angle_state[1,1],angle_state[2,1])
            llc.update_actual_local_velocities(position_state[0,1],position_state[1,1],position_state[2,1])

            #log rate rates in a log file
            np.savetxt(log, angle_state.reshape(1, -1), delimiter=',')

            if i == 1:
                fig, ax = plot_orientation(x=position_state[0,0], y=position_state[1,0], z=position_state[2,0], roll=angle_state[0,0], pitch=angle_state[1,0], yaw=angle_state[2,0], positions=x[0:3,:i], waypoints=[waypoints[0]['x'], waypoints[0]['y'], waypoints[0]['z']])
            elif i % 5 == 0:
                ax.clear()
                fig, ax = plot_orientation(fig, ax, x=position_state[0,0], y=position_state[1,0], z=position_state[2,0], roll=angle_state[0,0], pitch=angle_state[1,0], yaw=angle_state[2,0], positions=x[0:3,:i], waypoints=[waypoints[0]['x'], waypoints[0]['y'], waypoints[0]['z']])
            plt.draw()  # Draw the updated plot
            plt.pause(0.00001)#(0.01-((time.time()-last_update)/1000))  # Pause to update the plot


    # Plot the data
    plot(t_s, x, waypoints=[waypoints[0]['x'], waypoints[0]['y'], waypoints[0]['z'], None, None, None])