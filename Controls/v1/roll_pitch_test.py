from single_agent_controller.controllers.low_level_ctrl_2 import LLC2
from single_agent_controller.controllers_utils.helpers import *

from single_agent_simulator.x.system_dynamics import *
from single_agent_simulator.x.ode_solver import *
from single_agent_simulator.y.helpers import *
from single_agent_simulator.y.six_dof_model import *


from single_agent_visualiser.vector_visualiser import *
from single_agent_visualiser.log_visualiser import *

from utils.waypoints import *
from utils import CONSTANTS
from utils.constants2 import *

import matplotlib.pyplot as plt
import numpy as np


if __name__ == "__main__":

    #empty log file
    open('total_state.log', 'w').close()

    #initialize llc with initial state and store time 
    llc = LLC2(CONSTANTS['pid_params'], CONSTANTS['init_params'])
    last_update = time.time()

    #3x2 array for roll, pitch, yaw and their rates from the init_params
    angle_state = np.array([
        np.array([CONSTANTS['init_params']['roll_init'], CONSTANTS['init_params']['roll_rate_init']]),#, CONSTANTS['init_params']['roll_acc_init']]), 
        np.array([CONSTANTS['init_params']['pitch_init'], CONSTANTS['init_params']['pitch_rate_init']]),#, CONSTANTS['init_params']['pitch_acc_init']]), 
        np.array([CONSTANTS['init_params']['yaw_init'], CONSTANTS['init_params']['yaw_rate_init']])#, CONSTANTS['init_params']['yaw_acc_init']])
    ])

    with open('total_state.log', 'ab') as log:
    
        vehicle_model, t_s, h_s, x = initialize()
        x[3,0] = angle_state[0,0]


        for i in range(1,1000): #planner updates

            #update angle PIDs
            llc.update_angle_pids()

            #update angle rate PIDs
            torquex, torquey, torquez = llc.update_angle_rate_pids()
            print(torquex,torquey,torquez)
            
            x[15] = torquex
            x[16] = torquey
            x[16] = torquez


            #convert torques to angular accelerations:

            
            k1 = state_equations(x[:,i-1], vehicle_model)
            k2 = state_equations(x[:,i-1] + h_s*k1/2, vehicle_model)
            k3 = state_equations(x[:,i-1] + h_s*k2/2, vehicle_model)
            k4 = state_equations(x[:,i-1] + h_s*k3, vehicle_model)

            x[:,i] = x[:,i-1] + (h_s/6)*(k1+2*k2+2*k3+k4)

            #angle_state[0,2] = 0#todo
            #angle_state[1,2] = 0#todo
            #angle_state[2,2] = 0#todo

            angle_state[0,:] = np.array(x[3,i], x[9,i]) #np.array([0,0])#rk4(complex_system_dynamics, angle_state[0,1:], torquex, SIM_FREQ)
            angle_state[1,:] = np.array(x[4,i], x[10,i]) #np.array([0,0])#rk4(complex_system_dynamics, angle_state[1,1:], torquey, SIM_FREQ)
            angle_state[2,:] = np.array(x[5,i], x[11,i]) #np.array([0,0])#rk4(complex_system_dynamics, angle_state[2,1:], torquez, SIM_FREQ)


            #update angle state in controller
            llc.update_global_orientation_w_state(angle_state[2,0],angle_state[1,0],angle_state[0,0])

            #updae angle rate state in controller
            llc.update_actual_local_rates(angle_state[0,1],angle_state[1,1],angle_state[2,1])

            #log rate rates in a log file
            np.savetxt(log, angle_state.reshape(1, -1), delimiter=',')

                                
    # Load the logged data
    data = np.loadtxt('total_state.log', delimiter=',')
    time_p = np.arange(len(data))

    # Plot the data
    plot(t_s, x)

    #sanity check: yaw angle should be constant, yaw rate & z torque should be zero
    