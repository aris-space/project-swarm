from sac_.controllers.llc_v1 import LLC
from sac_.utils.helpers import *
from sac_.config.constants import *
from sac_.utils.states import *
import os
import matplotlib.pyplot as plt
import numpy as np
from sim_.x.six_dof import *
from sim_.x.single_rk4 import *
from _utils.waypoints import *

if __name__ == "__main__":

    #empty log file
    open('actual_state.log', 'w').close()

    pid_params = load_yaml('main/Controls/v1/sac_/config/pid_params.yaml')
    init_params = load_yaml('main/Controls/v1/_config/constants.yaml')['initial_orientation']


    #initialize llc with initial state
    llc = LLC(pid_params, init_params, llc_freq)
    #3x2 array for roll, pitch, yaw and their rates
    #create a 3x2 array for roll, pitch, yaw and their rates from the init_params
    actual_state = np.array([[init_params['roll_init'], init_params['roll_rate_init']], [init_params['pitch_init'], init_params['pitch_rate_init']], [init_params['yaw_init'], init_params['yaw_rate_init']]])

    prev_torques = np.zeros(3)

    #initialize vehicle model

    with open('actual_state.log', 'ab') as f:

        for i in range(1):#range(num_planner_updates): #planner updates

            #update target state
            llc.update_target_state(waypoints[i])

            for _ in range(100):#range(int(loc_freq / planner_freq)): #loc updates

                for _ in range(imu_freq // loc_freq): #imu updates

                    #give actual rates from sim to controller
                    llc.update_from_IMU_np_arr(actual_state)

                    for _ in range(llc_freq // imu_freq): #controller updates

                        #calculate desired rates from desired angles
                        llc.update_desired_arates()

                        #calculate torques from desired rates
                        torquex,torquey,torquez = llc.update_torques()

                        for _ in range(sim_freq//llc_freq): #sim updates
                            #update angles with RK4
                            actual_state[0,:] = advanced_rk4(system_dynamics, actual_state[0], prev_torques[0], torquex, sim_freq)
                            actual_state[1,:] = advanced_rk4(system_dynamics, actual_state[1], prev_torques[1], torquey, sim_freq)
                            actual_state[2,:] = advanced_rk4(system_dynamics, actual_state[2], prev_torques[2], torquez, sim_freq)
                            #measure actual rates

                            #log actual rates in a log file
                            
                            np.savetxt(f, actual_state.reshape(1, -1), delimiter=',')

                            prev_torques = [torquex, torquey, torquez]


    #plot actual_state as dots
    # Load the logged data
    data = np.loadtxt('actual_state.log', delimiter=',')

    time = np.arange(len(data))
    plt.plot(time, data)
    plt.xlabel('Time (line index)')
    plt.ylabel('Actual State Variables')
    plt.title('Actual State over Time')
    plt.legend(['Roll Angle', 'Roll Rate', 'Pitch Angle', 'Pitch Rate', 'Yaw Angle', 'Yaw Rate'])
    plt.show()



    