from controllers.LLC import LLC
from controllers.PID import PID
import yaml
import time

def load_config(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

if __name__ == "__main__":
    # Load configurations
    pid_params = load_config("config/PID_params.yaml")
    llc_config = load_config("config/LLC_config.yaml")

    current_state = {'depth': 10, 'depth_rate': 0.0}
    target_state = {'depth': 12, 'depth_rate': 0.0}
    dt = 0.001  # Time step

    # Initialize 6-DOF Controller
    llc = LLC(pid_params)

    with open("log.txt", "w") as log_file:
        #Update the controller for 100 iterations
        for _ in range(5000):  # Run for 100 iterations as an example
            thrust_z = llc.update(current_state, target_state, dt)

            log_file.write(f"Current State: {current_state}, Thrust in z direction: {thrust_z}\n")
            
            # Simulate updating the current state (this should be replaced with actual sensor data)
            current_state['depth'] += current_state['depth_rate'] * dt
            current_state['depth_rate'] += thrust_z * dt
            
            # Wait for the next time step
            time.sleep(dt)