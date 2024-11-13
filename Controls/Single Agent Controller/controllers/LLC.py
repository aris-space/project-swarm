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
    
    def update_IMU(self, state):

        self.roll_ctrl.update_cda(state['roll'])
        self.roll_ctrl.update_dar()

        self.roll_ctrl.update_cdar(state['droll'])


        self.pitch_ctrl.update_cda(state['pitch'])
        self.pitch_ctrl.update_dar()

        self.pitch_ctrl.update_cdar(state['dpitch'])


        self.yaw_ctrl.update_cda(state['yaw'])
        self.yaw_ctrl.update_dar()

        self.yaw_ctrl.update_cdar(state['dyaw'])

        self.depth_ctrl.update_cdd(state['z'])
        self.depth_ctrl.update_ddr()

        self.depth_ctrl.update_cddr(state['dz'])

    def update_loc(self, state):
        #update loc data
        pass

    def update_torques(self):
        # Update each PID based on current state and target state
        torque_y = self.roll_ctrl.update_dtau()
        torque_x = self.pitch_ctrl.update_dtau()
        torque_z = self.yaw_ctrl.update_dtau()
        return torque_y, torque_x, torque_z
    
    def update_thrust_z(self):
        # Update depth PID based on current state and target state
        thrust_z = self.depth_ctrl.update_dt()
        return thrust_z
    
    def check_orientation():
        # Check if the vehicle is oriented correctly
        return False
    
    def update_thrust_x(self):
        pass

    def update_target_state(self, target_state):
        # Update target state for each PID
        self.depth_ctrl.update_dd(target_state['z'])
        self.roll_ctrl.update_da(target_state['roll'])
        self.pitch_ctrl.update_da(target_state['pitch'])
        self.yaw_ctrl.update_da(target_state['yaw'])

    def update_desired_arates(self):
        # Update desired angles and rates for each PID
        self.roll_ctrl.update_dar()
        self.pitch_ctrl.update_dar()
        self.yaw_ctrl.update_dar()

    def update_desired_drate(self):
        # Update desired angles and rates for each PID
        self.depth_ctrl.update_ddr()
