from sac_.controllers.pid_v1 import PID
from sac_.controllers.depth_ctrl import depth_ctrl
from sac_.controllers.angle_ctrl import angle_ctrl
import os
import yaml
from sac_.config.constants import *
from scipy.spatial.transform import Rotation as R
import numpy as np

# Construct the relative path to the YAML file
yaml_path = os.path.join(os.path.dirname(__file__), "../config/pid_params.yaml")

with open(yaml_path, "r") as file:
    pid_params = yaml.safe_load(file)

# Access specific controller parameters
#depth_controller_params = pid_params["pid_params"]["depth_controller"]

class LLC:

    def __init__(self, pid_params, llc_freq, roll, pitch, yaw):
        # Initialize six PIDs for each degree of freedom
        #self.depth_ctrl = depth_ctrl(pid_params['depth'], llc_freq)

        self.roll_ctrl = angle_ctrl(pid_params['roll'], llc_freq)
        self.pitch_ctrl = angle_ctrl(pid_params['pitch'], llc_freq)
        self.yaw_ctrl = angle_ctrl(pid_params['yaw'], llc_freq)
        
        self.orientation_estimate_quat = self.euler_to_quaternion(self, roll, pitch, yaw)  # Initial orientation quaternion

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Converts Euler angles to a quaternion.

        Args:
            roll (float): Roll angle (rad).
            pitch (float): Pitch angle (rad).
            yaw (float): Yaw angle (rad).

        Returns:
            np.ndarray: Quaternion (x, y, z, w).
        """
        # Compute the quaternion
        q = R.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_quat()
        return q

    def update_orientation(global_quat, roll_rate, pitch_rate, yaw_rate):
        """
        Updates the global orientation quaternion based on local angular rates.

        Args:
            global_quat (np.ndarray): The current global orientation quaternion (x, y, z, w).
            roll_rate (float): Angular velocity around the x-axis (rad/s).
            pitch_rate (float): Angular velocity around the y-axis (rad/s).
            yaw_rate (float): Angular velocity around the z-axis (rad/s).
            dt (float): Time step for integration (s).

        Returns:
            np.ndarray: Updated global orientation quaternion (x, y, z, w).
            float, float, float: Updated global Euler angles roll, pitch, yaw (rad).
        """
        # Step 1: Compute the magnitude of angular velocity (|ω| = sqrt(roll_rate^2 + pitch_rate^2 + yaw_rate^2))
        omega_mag = np.sqrt(roll_rate**2 + pitch_rate**2 + yaw_rate**2)
        
        # Step 2: If angular velocity is very small, treat it as no rotation
        if omega_mag < 1e-8:
            roll, pitch, yaw = R.from_quat(global_quat).as_euler('xyz', degrees=False)
            return global_quat, roll, pitch, yaw
        
        # Step 3: Compute the incremental rotation quaternion (local frame)
        theta = omega_mag * 1/imu_freq  # Angle of rotation
        axis = np.array([roll_rate, pitch_rate, yaw_rate]) / omega_mag  # Rotation axis
        delta_quat = R.from_rotvec(axis * theta).as_quat()  # Incremental quaternion
        
        # Step 4: Update the global quaternion by applying the incremental rotation
        global_rotation = R.from_quat(global_quat)  # Convert global_quat to a Rotation object
        incremental_rotation = R.from_quat(delta_quat)  # Incremental rotation quaternion
        new_global_rotation = global_rotation * incremental_rotation  # Combine rotations
        
        # Step 5: Convert back to quaternion and Euler angles
        new_global_quat = new_global_rotation.as_quat()  # Updated quaternion (x, y, z, w)
        roll, pitch, yaw = new_global_rotation.as_euler('xyz', degrees=False)  # Roll, pitch, yaw
        
        return new_global_quat, roll, pitch, yaw
    
    def update_IMU_np_vec(self, state: np.ndarray):
        roll_rate = state[9]
        pitch_rate = state[10]
        yaw_rate = state[11]
        
        self.orientation_estimate_quat, roll, pitch, yaw = self.update_orientation(self.orientation_estimate_quat, roll_rate, pitch_rate, yaw_rate)

        self.roll_ctrl.update_cda(roll)
        self.roll_ctrl.update_dar()

        self.roll_ctrl.update_cdar(roll_rate)


        self.pitch_ctrl.update_cda(pitch)
        self.pitch_ctrl.update_dar()

        self.pitch_ctrl.update_cdar(pitch_rate)


        self.yaw_ctrl.update_cda(yaw)
        self.yaw_ctrl.update_dar()

        self.yaw_ctrl.update_cdar(yaw_rate)


        """
        self.depth_ctrl.update_cdd(z)
        self.depth_ctrl.update_ddr()

        self.depth_ctrl.update_cddr(z_rate)
        """

    def update_IMU_dict(self, state):

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
    
    def check_orientation(self):
        # Check if the vehicle is oriented correctly
        if (self.roll_ctrl.current_detectable_angle - self.roll_ctrl.desired_angle <= deg_margin and self.pitch_ctrl.current_detectable_angle - self.pitch_ctrl.desired_angle <= deg_margin and self.yaw_ctrl.current_detectable_angle - self.yaw_ctrl.desired_angle <= deg_margin):
            print(deg_margin)
            return True
        return False


    def update_target_state(self, target_state):
        # Update target state for each PID
        if target_state[2] is not None:
            self.depth_ctrl.update_dd(target_state['z'])
        if target_state['roll'] is not None:
            self.roll_ctrl.update_da(target_state['roll'])
        if target_state['pitch'] is not None:
            self.pitch_ctrl.update_da(target_state['pitch'])
        if target_state['yaw'] is not None:
            self.yaw_ctrl.update_da(target_state['yaw'])
        

    def update_desired_arates(self):
        # Update desired angles and rates for each PID
        self.roll_ctrl.update_dar()
        self.pitch_ctrl.update_dar()
        self.yaw_ctrl.update_dar()

    def update_desired_drate(self):
        # Update desired angles and rates for each PID
        self.depth_ctrl.update_ddr()
