from controllers.llc_v1 import LLC
from controllers.pid_v1 import PID
from utils.helpers import *
from config.constants import *
from visualisation.waypoints import *
from utils.states import *
import yaml
import time
import os
import matplotlib.pyplot as plt
import numpy as np



if __name__ == "__main__":
    
    # Initialize paths & load config files
    base_dir = os.path.dirname(os.path.abspath(__file__))
    pid_params_path, llc_config_path, log_file_path = initialize_paths(base_dir)

    pid_params = load_config(pid_params_path)
    llc_config = load_config(llc_config_path)

    # Initialize initial state
    states = append_state(states, roll=5, pitch=5, z=10)

    # Initialize Time Steps
    runtime = num_planner_updates // planner_freq
    t_planner, t_loc, t_imu, t_llc = initialize_time_steps(planner_freq, loc_freq, imu_freq, llc_freq)

    # Initialize LLC
    llc = LLC(pid_params, llc_freq)

    # Run the simulation

    with open(log_file_path, "w") as log_file:
        for i in range(1):#range(num_planner_updates):
            
            #update target state
            llc.update_target_state(waypoints[i])

            for j in range(1):#range(int(loc_freq / planner_freq)):

                #update loc data
                #llc.update_loc(states[-1])
                pass

                for _ in range(50):#range(imu_freq // loc_freq):

                    #update imu data
                    llc.update_IMU(states[-1])

                    for _ in range(50):#range(llc_freq // imu_freq):

                        #update the angles and rates by adding a new row and multiplying the torque from the step before with the time step
                        states = next_state(states, t_llc)

                        #calculate desired rates
                        llc.update_desired_arates()
                        llc.update_desired_drate()

                        #calculate torques
                        torquey,torquex,torquez = llc.update_torques()
                        #calculate thrust in z direction
                        thrustz = llc.update_thrust_z()
                        #depth = llc.update_depth()
                        """
                        #check if the orientation is correct and if yes, compute desired dx and then thrust
                        if llc.check_orientation():
                            pass
                        else:
                        """

                        


                        #add torques to the current state                  
                        states[-1]['torquey'] = torquey
                        states[-1]['torquex'] = torquex
                        states[-1]['torquez'] = torquez

                        #add thrustz to the current state
                        states[-1]['thrustz'] = thrustz


                        #store state[-1] in log file in new line
                        log_file.write(f"{states[-1]}\n")
                        

                        #llc.update_angles_and_rates()

    
    plot_results(states, log_file_path)

    """
    #pseudocode

    check if all angles are within threshold
    if yes, compute desired dx and then thrust
    if no, set thrust in x direction to 0 and continue

    """

    