from controllers.llc import LLC
from controllers.pid import PID
import yaml
import time
import os
import matplotlib.pyplot as plt

def load_config(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

if __name__ == "__main__":
    # Construct absolute paths
    base_dir = os.path.dirname(os.path.abspath(__file__))
    pid_params_path = os.path.join(base_dir, 'config', 'pid_params.yaml')
    llc_config_path = os.path.join(base_dir, 'config', 'llc_config.yaml')
    log_file_path = os.path.join(base_dir, 'log.txt')

    # Load configurations
    pid_params = load_config(pid_params_path)
    llc_config = load_config(llc_config_path)

    current_state = {'depth': -10, 'depth_rate': 0.0}
    dt = 0.001  # Time step

    # Initialize 6-DOF Controller
    llc = LLC(pid_params)
            

    planner_freq = 0.1
    t_planner = (1/planner_freq)
    loc_freq = 5
    t_loc = (1/loc_freq)
    imu_freq = 200
    t_imu = (1/imu_freq)
    llc_freq = 400
    t_llc = (1/llc_freq)

    print(f"Planner freq: {planner_freq}, Loc freq: {loc_freq}, IMU freq: {imu_freq}, LLC freq: {llc_freq}")
    print(f"t_planner: {t_planner}, t_loc: {t_loc}, t_imu: {t_imu}, t_llc: {t_llc}")
    
    num_planner_updates = 1
    runtime = num_planner_updates//planner_freq

    current_detectable_depth_state = {}
    depths = [] 
    detectable_depths = []

    with open(log_file_path, "w") as log_file:
        for _ in range(num_planner_updates): #planner update

            target_state = {'depth': -20}
            llc.depth_ctrl.update_dd(target_state['depth'])

            for _ in range (int(loc_freq/planner_freq)): #loc_update

                current_detectable_depth_state['depth'] = current_state['depth']
                llc.depth_ctrl.update_cd(current_detectable_depth_state['depth'])
                llc.depth_ctrl.update_ddr()

                for _ in range(imu_freq//loc_freq):

                    current_detectable_depth_state['depth_rate'] = current_state['depth_rate']
                    llc.depth_ctrl.update_cdr(current_detectable_depth_state['depth_rate'])

                    for _ in range(llc_freq//imu_freq):

                        thrust_z = llc.depth_ctrl.update_dtz()

                        #write the current state to a dictionary with key as time

                        log_file.write(f"Current Depth: {current_state['depth']}, Current Depth Rate: {current_state['depth_rate']}, Thrust in z direction: {thrust_z}, desired depth: {llc.depth_ctrl.desired_depth}, desired depth rate: {llc.depth_ctrl.desired_depth_rate}\n")
                        
                        depths.append(current_state['depth'])
                        detectable_depths.append(current_detectable_depth_state['depth'])

                        #updatng current state
                        current_state['depth'] += current_state['depth_rate'] * t_llc
                        current_state['depth_rate'] += thrust_z * t_llc
                        
                        # Wait for the next time step
                        #time.sleep(t_llc)


    times = [i for i in range(len(depths))]


    plt.plot(times, detectable_depths, label='Detectable Depth')
    plt.legend()
    plt.figure()
    plt.plot(times, depths, label='Depth')
    plt.xlabel('Time (ms)')
    plt.ylabel('Depth (m)')
    plt.title('Depth over Time')
    plt.grid(True)
    plt.show()