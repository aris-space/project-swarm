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
    states = append_state(states, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    # Initialize Time Steps
    runtime = num_planner_updates // planner_freq
    t_planner, t_loc, t_imu, t_llc = initialize_time_steps(planner_freq, loc_freq, imu_freq, llc_freq)

    # Initialize LLC
    llc = LLC(pid_params, llc_freq)

    # Run the simulation

    with open(log_file_path, "w") as log_file:
        for i in range(num_planner_updates):
            
            #update target state
            llc.update_target_state(waypoints[i])

            for j in range(int(loc_freq / planner_freq)):

                #update loc data
                llc.update_loc(states[-1])

                for _ in range(imu_freq // loc_freq):
                    llc.depth_ctrl.current_detectable_depth_state['depth_rate'] = states[-1]['dz']
                    llc.depth_ctrl.update_cdr(llc.depth_ctrl.current_detectable_depth_state['depth_rate'])

                    llc.roll_ctrl.current_detectable_angle_state['roll'] = states[-1]['roll']
                    llc.roll_ctrl.update_cd(llc.roll_ctrl.current_detectable_angle_state['roll'])
                    llc.roll_ctrl.update_ddr()

                    llc.roll_ctrl.current_detectable_angle_state['roll_rate'] = states[-1]['droll']
                    llc.roll_ctrl.update_cdr(llc.roll_ctrl.current_detectable_angle_state['roll_rate'])

                    for _ in range(llc_freq // imu_freq):
                        thrust_z = llc.depth_ctrl.update_dtz()
                        torque_y = llc.roll_ctrl.update_dtau()

                        log_state(log_file, states[-1], thrust_z, llc)

                        llc.roll_ctrl.angles.append(states[-1]['roll'])
                        llc.roll_ctrl.detectable_angles.append(llc.roll_ctrl.current_detectable_angle_state['roll'])

                        llc.depth_ctrl.depths.append(states[-1]['z'])
                        llc.depth_ctrl.detectable_depths.append(llc.depth_ctrl.current_detectable_depth_state['depth'])

                        update_current_state(states[-1], thrust_z, torque_y, t_llc)

    times = [i for i in range(len(llc.depth_ctrl.depths))]
    plot_results(times, llc.roll_ctrl.detectable_angles, llc.roll_ctrl.angles, llc)