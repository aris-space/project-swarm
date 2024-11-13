from controllers.pid import PID
from controllers.depth_ctrl import depth_ctrl
from controllers.angle_ctrl import angle_ctrl
import os
import yaml

# Construct the relative path to the YAML file
yaml_path = os.path.join(os.path.dirname(__file__), "../config/pid_params.yaml")

with open(yaml_path, "r") as file:
    pid_params = yaml.safe_load(file)

# Access specific controller parameters
#depth_controller_params = pid_params["pid_params"]["depth_controller"]

class LLC:
    def __init__(self, pid_params, llc_freq):
        # Initialize six PIDs for each degree of freedom
        self.depth_ctrl = depth_ctrl(pid_params['depth'], llc_freq)
        self.roll_ctrl = angle_ctrl(pid_params['roll'], llc_freq)
        self.pitch_ctrl = angle_ctrl(pid_params['pitch'], llc_freq)
        self.yaw_ctrl = angle_ctrl(pid_params['yaw'], llc_freq)
    
    def fetch_IMU(self):
        pass

    def fetch_loc(self):
        pass

    def update_angles(self, target_state, current_state):
        # Update each PID based on current state and target state
        torque_y = self.roll_ctrl.update_all()
        torque_x = self.pitch_ctrl.update_all()
        torque_z = self.yaw_ctrl.update_all()
        return torque_y, torque_x, torque_z

    def update_all(self, target_state, current_state):
        # Update each PID based on current state and target state
        thrust_z = self.depth_ctrl.update_all(target_state, current_state)        
        return thrust_z
    
    def update_depth(self, target_state, current_state):
        # Update depth PID based on current state and target state
        thrust_z = self.depth_ctrl.update_all(target_state, current_state)
        return thrust_z
    
    def update_forwards():
        pass

    def check_orientation():
        pass

