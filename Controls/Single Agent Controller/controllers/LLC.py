from .PID import PID
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
    
    def update(self, current_state, target_state, dt):
        # Update each PID based on current state and target state
        thrust_z = self.depth_ctrl.update(current_state, target_state, dt)        
        return thrust_z

class depth_ctrl:
    def __init__(self, pid_params):
        self.depth_controller = PID(**pid_params['depth_controller'])
        self.depth_rate_controller = PID(**pid_params['depth_controller'])
        
    def update(self, current_state, target_state, dt):
        # Get current and target depths
        current_depth = current_state['depth']
        desired_depth = target_state['depth']

        # Outer loop PID: depth controller calculates desired depth rate
        desired_depth_rate = self.depth_controller.update(desired_depth, current_depth, dt)
        print(f"Desired Depth Rate: {desired_depth_rate}")

        #print(f"Desired Depth Rate: {desired_depth_rate}")
        #print(f"Current Depth Rate: {current_state['depth_rate']}")

        # Get current depth rate
        current_depth_rate = current_state['depth_rate']

        # Inner loop PID: depth rate controller calculates Thrust in z direction
        #depth_rate_error = desired_depth_rate - current_depth_rate

        #print(f"depth_rate_error: {depth_rate_error}")

        thrust_z = self.depth_rate_controller.update(desired_depth_rate, current_depth_rate, dt)

        return thrust_z