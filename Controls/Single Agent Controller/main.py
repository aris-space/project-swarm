from controllers.LLC import LLC
from controllers.PID import PID
import yaml
import time
import os

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

    current_state = {'depth': 10, 'depth_rate': 0.0}
    target_state = {'depth': 12}
    dt = 0.001  # Time step

    # Initialize 6-DOF Controller
    llc = LLC(pid_params)

    """

    with open("log.txt", "w") as log_file:

        for _ in range(5000):  # Run for 100 iterations as an example
            thrust_z = llc.update_all(target_state, current_state)

            log_file.write(f"Current State: {current_state}, Thrust in z direction: {thrust_z}\n")
            
            # Simulate updating the current state (this should be replaced with actual sensor data)
            current_state['depth'] += current_state['depth_rate'] * dt
            current_state['depth_rate'] += thrust_z * dt
            
            # Wait for the next time step
            time.sleep(dt)

    """
            

    planner_freq = 0.2
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

    with open(log_file_path, "w") as log_file:
        for _ in range(num_planner_updates): #planner_freq*runtime
            for _ in range (int(loc_freq/planner_freq)):
                for _ in range(imu_freq//loc_freq):
                    for _ in range(llc_freq//imu_freq):
                        thrust_z = llc.update_all(target_state, current_state)
                        log_file.write(f"Current State: {current_state}, Thrust in z direction: {thrust_z}, desired depth: {llc.depth_ctrl.desired_depth}, desired depth rate: {llc.depth_ctrl.desired_depth_rate}\n")
                        #updatng current state
                        current_state['depth'] += current_state['depth_rate'] * t_llc
                        current_state['depth_rate'] += thrust_z * t_llc
                        # Wait for the next time step
                        time.sleep(t_llc)

    