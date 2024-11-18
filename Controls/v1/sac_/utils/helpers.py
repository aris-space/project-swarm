from sac_.controllers.llc_v1 import LLC
from sac_.controllers.pid_v1 import PID
import yaml
import time
import os
import matplotlib.pyplot as plt
from sac_.config.constants import *
import numpy as np


def load_config(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def initialize_paths(base_dir):
    pid_params_path = os.path.join(base_dir, 'sac_', 'config', 'pid_params.yaml')
    llc_config_path = os.path.join(base_dir, 'sac_', 'config', 'llc_config.yaml')
    log_file_path = os.path.join(base_dir, 'log.txt')
    return pid_params_path, llc_config_path, log_file_path

def initialize_frequencies(file_path):
    #load frequencies from yaml file and return them
    with open(file_path, "r") as file:
        freqs = yaml.safe_load(file)
    planner_freq = freqs['planner_freq']
    loc_freq = freqs['loc_freq']
    imu_freq = freqs['imu_freq']
    llc_freq = freqs['llc_freq']
    return planner_freq, loc_freq, imu_freq, llc_freq

def initialize_time_steps(planner_freq, loc_freq, imu_freq, llc_freq):
    t_planner = 1 / planner_freq
    t_loc = 1 / loc_freq
    t_imu = 1 / imu_freq
    t_llc = 1 / llc_freq
    return t_planner, t_loc, t_imu, t_llc

def log_state(log_file, current_state, thrust_z, llc):
    log_file.write(f"Current Depth: {current_state['z']}, Current Depth Rate: {current_state['dz']}, "
                   f"Thrust in z direction: {thrust_z}, desired depth: {llc.depth_ctrl.desired_depth}, "
                   f"desired depth rate: {llc.depth_ctrl.desired_depth_rate}\n")



def append_state(array, x=0, y=0, z=0, roll=0, pitch=0, yaw=0, dx=0, dy=0, dz=0, droll=0, dpitch=0, dyaw=0, thrustx=0, thrusty=0, thrustz=0, torquex=0, torquey=0, torquez=0):
    # Create a dictionary with 6-DOF and derivatives
    new_row = {
        'x': x, 'y': y, 'z': z,
        'roll': roll, 'pitch': pitch, 'yaw': yaw,
        'dx': dx, 'dy': dy, 'dz': dz,
        'droll': droll, 'dpitch': dpitch, 'dyaw': dyaw,
        'thrustx': thrustx, 'thrusty': thrusty, 'thrustz': thrustz,
        'torquex': torquex, 'torquey': torquey, 'torquez': torquez
    }

    #print(new_row)
    
    # Append the new row (dictionary) to the NumPy array
    array = np.append(array, new_row)
    
    return array

def next_state(states, t_llc):
    new_state = states[-1].copy()
    states = np.append(states, new_state)

    states[-1]['droll'] = states[-2]['droll'] + states[-2]['torquey'] * t_llc
    states[-1]['roll'] = states[-2]['roll'] + (states[-2]['droll'] + states[-2]['torquey'] * t_llc) * t_llc
    states[-1]['dpitch'] = states[-2]['dpitch'] + states[-2]['torquex'] * t_llc
    states[-1]['pitch'] = states[-2]['pitch'] + (states[-2]['dpitch'] + states[-2]['torquex'] * t_llc) * t_llc
    states[-1]['dyaw'] = states[-2]['dyaw'] + states[-2]['torquez'] * t_llc
    states[-1]['yaw'] = states[-2]['yaw'] + (states[-2]['dyaw'] + states[-2]['torquez'] * t_llc) * t_llc
    states[-1]['dz'] = states[-2]['dz'] + states[-2]['thrustz'] * t_llc
    states[-1]['z'] = states[-2]['z'] + (states[-2]['dz'] + states[-2]['thrustz'] * t_llc) * t_llc


    return states

def plot_results(states, log_file_path):
        # Read the log file and extract roll values
    timestamps = []
    roll_values = []
    yaw_values = []
    pitch_values = []
    depth_values = []

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
            pitch_values.append(state['pitch'])
            yaw_values.append(state['yaw'])
            depth_values.append(state['z'])
            i+=1

    # Plot roll values over time
    plt.figure()
    plt.plot(timestamps, roll_values, label='Roll')
    plt.plot(timestamps, pitch_values, label='Pitch')
    plt.plot(timestamps, yaw_values, label='Yaw')
    plt.plot(timestamps, depth_values, label='Depth')
    plt.xlabel('Time (s)')
    plt.ylabel('Roll (degrees)')
    plt.title('Roll over Time')
    plt.legend()
    plt.grid(True)
    plt.show()