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

    # Load configurations
    pid_params = load_config(pid_params_path)
    llc_config = load_config(llc_config_path)

    current_state = {'depth': 10, 'depth_rate': 0.0}
    target_state = {'depth': 12, 'depth_rate': 0.0}
    dt = 0.001  # Time step

    # Initialize 6-DOF Controller
    llc = LLC(pid_params)

    with open("log.txt", "w") as log_file:
        #Update the controller for 100 iterations
        for _ in range(5000):  # Run for 100 iterations as an example
            thrust_z = llc.update_all(target_state, current_state)

            log_file.write(f"Current State: {current_state}, Thrust in z direction: {thrust_z}\n")
            
            # Simulate updating the current state (this should be replaced with actual sensor data)
            current_state['depth'] += current_state['depth_rate'] * dt
            current_state['depth_rate'] += thrust_z * dt
            
            # Wait for the next time step
            time.sleep(dt)