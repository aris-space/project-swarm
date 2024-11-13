from controllers.pid import PID
from controllers.depth_ctrl import depth_ctrl
from controllers.roll_ctrl import roll_ctrl
import os
import yaml

# Construct the relative path to the YAML file
yaml_path = os.path.join(os.path.dirname(__file__), "../config/pid_params.yaml")

with open(yaml_path, "r") as file:
    pid_params = yaml.safe_load(file)

# Access specific controller parameters
#depth_controller_params = pid_params["pid_params"]["depth_controller"]

class LLC:
    def __init__(self, pid_params):
        # Initialize six PIDs for each degree of freedom
        self.depth_ctrl = depth_ctrl(pid_params)
        self.roll_ctrl = roll_ctrl(pid_params)
    
    def update_IMU(self):
        pass

    def update_all(self, target_state, current_state):
        # Update each PID based on current state and target state
        thrust_z = self.depth_ctrl.update_all(target_state, current_state)        
        return thrust_z

