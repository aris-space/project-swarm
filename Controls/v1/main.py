from single_agent_controller.controllers.low_level_ctrl import LLC
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

# needed for plotting
plt.ion()
fig = None
ax = None

if __name__ == "__main__":

    #empty log file
    open('total_state.log', 'w').close()


    #initialize llc with initial state
    llc = LLC(CONSTANTS['pid_params'], CONSTANTS['init_params'], LLC_FREQ)
    #3x2 array for roll, pitch, yaw and their rates from the init_params
    angle_state = np.array([[CONSTANTS['init_params']['roll_init'], CONSTANTS['init_params']['roll_rate_init']], [CONSTANTS['init_params']['pitch_init'], CONSTANTS['init_params']['pitch_rate_init']], [CONSTANTS['init_params']['yaw_init'], CONSTANTS['init_params']['yaw_rate_init']]])
    #1x2 array for depth and rate from the init_params
    depth_state = np.array([CONSTANTS['init_params']['depth_init'], CONSTANTS['init_params']['depth_rate_init']])

    #initialize short term thrust and torques memory (needed for RK4/Euler)
    prev_torques = np.zeros(3)
    prev_thrusts = np.zeros(3)

    with open('total_state.log', 'ab') as f:

        for i in range(NUM_PLANNER_UPDATES): #planner updates

            #measure how long it takes for one iteration
            start = time.time()


            #update target state
            llc.update_target_state(waypoints[i])

            for _ in range(int(LOC_FREQ / PLANNER_FREQ)): #loc updates


                for _ in range(PRES_FREQ // LOC_FREQ): #pres updates

                    #give pressure from pres to controller
                    llc.update_from_pres_np_arr(depth_state[0])

                    for _ in range(IMU_FREQ // PRES_FREQ): #imu updates

                        #plot orientation => just here because fps is about right
                        fig, ax = plot_orientation(fig, ax, 0, 0, depth_state[0], angle_state[0, 0], angle_state[1, 0], angle_state[2, 0])
        

                        #give angle rates from sim to controller
                        llc.update_from_IMU_np_arr(angle_state, depth_state)

                        for _ in range(LLC_FREQ // IMU_FREQ): #controller updates

                            #calculate desired rates from desired angles
                            llc.update_desired_arates()

                            #calculate torques from desired rates
                            torquex,torquey,torquez = llc.update_torques()

                            #calculate thrust from desired depth rate
                            thrustz = llc.update_thrust_z()

                            for _ in range(SIM_FREQ // LLC_FREQ): #sim updates
                                #update angles with RK4
                                angle_state[0,:] = rk4(simple_system_dynamics, angle_state[0], prev_torques[0], torquex, SIM_FREQ)
                                angle_state[1,:] = rk4(simple_system_dynamics, angle_state[1], prev_torques[1], torquey, SIM_FREQ)
                                angle_state[2,:] = rk4(simple_system_dynamics, angle_state[2], prev_torques[2], torquez, SIM_FREQ)
                                depth_state      = rk4(simple_system_dynamics, depth_state, prev_thrusts[2], thrustz, SIM_FREQ)
                                #measure angle rates

                                #log rate rates in a log file

                                #create a 4x2 array for roll, pitch, yaw and their rates and depth and rate
                                total_state = np.concatenate((angle_state, depth_state.reshape(1, -1)), axis=0)
                                
                                np.savetxt(f, total_state.reshape(1, -1), delimiter=',')

                                #time.sleep(1/SIM_FREQ/3)
                                

    #stop interactive plotting
    plt.ioff()

    end = time.time()
    print(f"Time taken for one iteration: {end - start}")
    print(RUNTIME)


    # Load the logged data
    data = np.loadtxt('total_state.log', delimiter=',')
    time_p = np.arange(len(data))

    # Plot the data
    log_visualiser(time_p, data)


    