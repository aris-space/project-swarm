from single_agent_controller.controllers.low_level_ctrl_2 import LLC2
from single_agent_controller.controllers_utils.helpers import *

from single_agent_simulator.x.system_dynamics import *
from single_agent_simulator.x.ode_solver import *

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
        np.array([CONSTANTS['init_params']['roll_init'], CONSTANTS['init_params']['roll_rate_init'], CONSTANTS['init_params']['roll_acc_init']]), 
        np.array([CONSTANTS['init_params']['pitch_init'], CONSTANTS['init_params']['pitch_rate_init'], CONSTANTS['init_params']['pitch_acc_init']]), 
        np.array([CONSTANTS['init_params']['yaw_init'], CONSTANTS['init_params']['yaw_rate_init'], CONSTANTS['init_params']['yaw_acc_init']])
    ])

    with open('total_state.log', 'ab') as log:

        for i in range(1000): #planner updates

            #update angle PID
            llc.update_angle_pids()

            #update angle rate PID
            torquex, torquey, torquez = llc.update_angle_rate_pids()
            print(torquex,torquey,torquez)

            #convert torques to angular accelerations:
            angle_state[0,2] = 0#todo
            angle_state[1,2] = 0#todo
            angle_state[2,2] = 0#todo

            angle_state[0,:2] = np.array([0,0])#rk4(complex_system_dynamics, angle_state[0,1:], torquex, SIM_FREQ)
            angle_state[1,:2] = np.array([0,0])#rk4(complex_system_dynamics, angle_state[1,1:], torquey, SIM_FREQ)
            angle_state[2,:2] = np.array([0,0])#rk4(complex_system_dynamics, angle_state[2,1:], torquez, SIM_FREQ)


            #log rate rates in a log file
            np.savetxt(log, angle_state.reshape(1, -1), delimiter=',')
                                
    # Load the logged data
    data = np.loadtxt('total_state.log', delimiter=',')
    time_p = np.arange(len(data))

    # Plot the data

    #sanity check: yaw angle should be constant, yaw rate & z torque should be zero