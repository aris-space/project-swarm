from controllers.llc import LLC
from controllers.pid import PID
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
    states = append_state(states, roll=5, pitch=5)

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
                llc.update_loc(states[-1])

                for _ in range(50):#range(imu_freq // loc_freq):

                    #update imu data
                    llc.update_IMU(states[-1])

                    for _ in range(50):#range(llc_freq // imu_freq):


                        #update the angles and rates by adding a new row and multiplying the torque from the step before with the time step
                        new_state = states[-1].copy()
                        states = np.append(states, new_state)

                        states[-1]['droll'] = states[-2]['droll'] + states[-2]['torquey'] * t_llc
                        states[-1]['roll'] = states[-2]['roll'] + (states[-2]['droll'] + states[-2]['torquey'] * t_llc) * t_llc
                        states[-1]['dpitch'] = states[-2]['dpitch'] + states[-2]['torquex'] * t_llc
                        states[-1]['pitch'] = states[-2]['pitch'] + (states[-2]['dpitch'] + states[-2]['torquex'] * t_llc) * t_llc
                        states[-1]['dyaw'] = states[-2]['dyaw'] + states[-2]['torquez'] * t_llc
                        states[-1]['yaw'] = states[-2]['yaw'] + (states[-2]['dyaw'] + states[-2]['torquez'] * t_llc) * t_llc

                        llc.update_desired_rates()

                        #update angles and z
                        torquey,torquex,torquez = llc.update_torques()
                        print("torquey: ", torquey)
                        print("torquex: ", torquex)
                        print("torquez: ", torquez)
                        #llc.update_depth()




                        #add torques to the current state
                        states[-1]['torquey'] = torquey
                        states[-1]['torquex'] = torquex
                        states[-1]['torquez'] = torquez

                        #store state[-1] in log file in new line
                        log_file.write(f"{states[-1]}\n")
                        

                        #llc.update_angles_and_rates()

    # Read the log file and extract roll values
    timestamps = []
    roll_values = []

    #store all values of each state in the states in a csv file
    with open('states.csv', 'w') as f:
        for state in states:
            f.write("%s\n" % state)


    with open(log_file_path, "r") as log_file:
        i = 0
        for line in log_file:
            state = eval(line.strip())
            timestamps.append(i)
            roll_values.append(state['roll'])
            i+=1

    # Plot roll values over time
    plt.figure()
    plt.plot(timestamps, roll_values, label='Roll')
    plt.xlabel('Time (s)')
    plt.ylabel('Roll (degrees)')
    plt.title('Roll over Time')
    plt.legend()
    plt.grid(True)
    plt.show()
    
    #plot_results(times, llc)
    #print(states)

    """
    #pseudocode

    check if all angles are within threshold
    if yes, compute desired dx and then thrust
    if no, set thrust in x direction to 0 and continue

    """