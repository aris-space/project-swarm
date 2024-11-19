from single_agent_controller.controllers.depth_ctrl import depth_ctrl
from single_agent_controller.controllers.angle_ctrl import angle_ctrl

from utils.constants2 import *

from scipy.spatial.transform import Rotation as R
import numpy as np


class LLC:

    def __init__(self, pid_params, init_params, llc_freq):
        # Initialize six PIDs for each degree of freedom
        #self.depth_ctrl = depth_ctrl(pid_params['depth'], llc_freq)

        self.roll_ctrl = angle_ctrl(pid_params['roll'], llc_freq)
        self.pitch_ctrl = angle_ctrl(pid_params['pitch'], llc_freq)
        self.yaw_ctrl = angle_ctrl(pid_params['yaw'], llc_freq)

        self.depth_ctrl = depth_ctrl(pid_params['depth'], llc_freq)
        
        self.orientation_estimate_quat = self.euler_to_quaternion(init_params['roll_init'], init_params['pitch_init'], init_params['yaw_init'])  # Initial orientation quaternion
        self.position_estimate_vect = np.zeros(3)  # Initial position vector

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float):
        """
        Converts Euler angles to a quaternion.

        Args:
            roll (float): Roll angle (rad).
            pitch (float): Pitch angle (rad).
            yaw (float): Yaw angle (rad).

        Returns:
            np.ndarray: Quaternion (x, y, z, w).
        """

        q = R.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_quat()
        return q

    def quaternion_to_euler(self, quat: np.ndarray):
        """
        Converts a quaternion to Euler angles.

        Args:
            quat (np.ndarray): Quaternion (x, y, z, w).

        Returns:
            float, float, float: Euler angles roll, pitch, yaw (rad).
        """

        euler = R.from_quat(quat).as_euler('xyz', degrees=False)
        return euler

    def update_orientation(self, global_quat: np.ndarray, roll_rate: float, pitch_rate: float, yaw_rate: float):
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
        
        # Step 3: Compute the incremental rotation quaternion (local frame) => could be improved by RK4
        theta = omega_mag * 1/IMU_FREQ  # Angle of rotation
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
    
    def update_position(self, state: np.ndarray):
        """
        Estimates the vehicle's position based on the current state.

        Args:
            state (np.ndarray): Current state vector [x, y, z, roll, pitch, yaw, dx, dy, dz, droll, dpitch, dyaw].

        Returns:
            np.ndarray: Estimated position vector [x, y, z].
        """

        self.position_estimate_vect = state[0:3]
    
    def update_from_IMU_np_arr(self, angle_state: np.ndarray, depth_state: np.ndarray):
        roll_rate = angle_state[0,1]
        pitch_rate = angle_state[1,1]
        yaw_rate = angle_state[2,1]

        depth_rate = depth_state[1]
        
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

        self.depth_ctrl.update_cddr(depth_rate)

    def update_from_pres_np_arr(self, z):

        self.position_estimate_vect[2] = z

        self.depth_ctrl.update_cdd(z)
        self.depth_ctrl.update_ddr()
        


    def update_from_IMU_dict(self, state): #unused



        self.roll_ctrl.update_cda(state['roll'])
        self.roll_ctrl.update_dar()

        self.roll_ctrl.update_cdar(state['droll'])


        self.pitch_ctrl.update_cda(state['pitch'])
        self.pitch_ctrl.update_dar()

        self.pitch_ctrl.update_cdar(state['dpitch'])


        self.yaw_ctrl.update_cda(state['yaw'])
        self.yaw_ctrl.update_dar()

        self.yaw_ctrl.update_cdar(state['dyaw'])

        #self.depth_ctrl.update_cdd(state['z'])
        #self.depth_ctrl.update_ddr()

        #self.depth_ctrl.update_cddr(state['dz'])

    def update_loc(self, state): #unused
        #update loc data
        pass

    def update_torques(self):
        # Update each PID based on current state and target state
        torque_x = self.roll_ctrl.update_dtau()
        torque_y = self.pitch_ctrl.update_dtau()
        torque_z = self.yaw_ctrl.update_dtau()
        return torque_x, torque_y, torque_z
    
    def update_thrust_z(self):
        # Update depth PID based on current state and target state
        thrust_z = self.depth_ctrl.update_dt()
        return thrust_z
    
    def check_orientation(self):
        # Check if the vehicle is oriented correctly by using orientation estimate quaternion

        # Step 1: Convert the orientation estimate quaternion to Euler angles
        roll, pitch, yaw = self.quaternion_to_euler(self.orientation_estimate_quat)

        # Step 2: Check if the vehicle is within the desired orientation margins
        roll_margin = np.abs(roll - self.roll_ctrl.desired_angle) < DEG_MARGIN
        pitch_margin = np.abs(pitch - self.pitch_ctrl.desired_angle) < DEG_MARGIN
        yaw_margin = np.abs(yaw - self.yaw_ctrl.desired_angle) < DEG_MARGIN

        if roll_margin and pitch_margin and yaw_margin:
            return True
        else:
            return False

    def update_target_state(self, target_state: dict):
        # Update target state for each PID
        #if target_state['z'] is not None:
        #    self.depth_ctrl.update_dd(target_state['z'])
        if target_state['roll'] is not None:
            self.roll_ctrl.update_da(target_state['roll'])
        if target_state['pitch'] is not None:
            self.pitch_ctrl.update_da(target_state['pitch'])
        if target_state['yaw'] is not None:
            self.yaw_ctrl.update_da(target_state['yaw']) 
        if target_state['z'] is not None:
            self.depth_ctrl.update_dd(target_state['z'])

    def update_desired_arates(self):
        # Update desired angles and rates for each PID
        self.roll_ctrl.update_dar()
        self.pitch_ctrl.update_dar()
        self.yaw_ctrl.update_dar()

    def update_desired_drate(self):
        # Update desired angles and rates for each PID
        self.depth_ctrl.update_ddr()

