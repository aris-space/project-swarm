from sac_.controllers.llc_v1 import LLC
from sac_.utils.helpers import *
from sac_.config.constants import *
from sac_.visualisation.waypoints import *
from sac_.utils.states import *
import os
import matplotlib.pyplot as plt
import numpy as np


if __name__ == "__main__":

    actual_rates = np.zeros(3)

    pid_params = load_yaml('main/Controls/v1/sac_/config/pid_params.yaml')
    init_params = load_yaml('main/Controls/v1/01config/constants.yaml')['initial_orientation']

    #initialize llc with initial state
    llc = LLC(pid_params, init_params, llc_freq)

    for i in range(num_planner_updates): #planner updates

        #update target state
        llc.update_target_state(waypoints[i])

        for j in range(int(loc_freq / planner_freq)): #loc updates
            pass

            for _ in range(imu_freq // loc_freq): #imu updates


                #give actual rates from sim to controller
                llc.update_from_IMU_np_arr(actual_rates)

                for _ in range(llc_freq // imu_freq): #controller updates

                    #calculate desired rates from desired angles
                    llc.update_desired_arates()

                    #calculate torques from desired rates
                    torquey,torquex,torquez = llc.update_torques()

                    for _ in range(sim_freq//llc_freq): #sim updates
                        #update angles with RK4
                        ...
                        #measure actual rates
                        actual_rates = ...

