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
        
        self.orientation_estimate_quat = self.rotvec_to_quaternion(init_params['roll_init'], init_params['pitch_init'], init_params['yaw_init'])  # Initial orientation quaternion
        self.position_estimate_vect = np.zeros(3)  # Initial position vector

    def rotvec_to_quaternion(self, roll: float, pitch: float, yaw: float):

        q = R.from_rotvec([roll, pitch, yaw], degrees=False).as_quat()
        return q

    def quaternion_to_rotvec(self, quat: np.ndarray):


        rotvec = R.from_quat(quat).as_rotvec(degrees=False)
        return rotvec

    def update_orientation(self, global_quat: np.ndarray, roll_rate: float, pitch_rate: float, yaw_rate: float, dt=float):
        """
        Updates the global orientation quaternion based on local angular rates.

        Args:
            global_quat (np.ndarray): The current global orientation quaternion (x, y, z, w).
            roll_rate (float): Angular velocity around the x-axis (rad/s).
            pitch_rate (float): Angular velocity around the y-axis (rad/s).
            yaw_rate (float): Angular velocity around the z-axis (rad/s).

        Returns:
            np.ndarray: Updated global orientation quaternion (x, y, z, w).
            float, float, float: Updated global Euler angles roll, pitch, yaw (rad).
        """
        # Step 1: Compute the magnitude of angular velocity (|ω| = sqrt(roll_rate^2 + pitch_rate^2 + yaw_rate^2))
        omega_mag = np.sqrt(roll_rate**2 + pitch_rate**2 + yaw_rate**2)
        
        # Step 2: If angular velocity is very small, treat it as no rotation
        if omega_mag < 1e-8:
            roll, pitch, yaw = R.from_quat(global_quat).as_rotvec(degrees=False)
            return global_quat, roll, pitch, yaw
        
        # Step 3: Compute the incremental rotation quaternion (local frame) => could be improved by RK4
        theta = omega_mag * dt  # Angle of rotation, local
        axis = np.array([roll_rate, pitch_rate, yaw_rate]) / omega_mag  # Rotation axis, local
        delta_quat = R.from_rotvec(axis * theta).as_quat()  # Incremental quaternion, local
        
        # Step 4: Update the global quaternion by applying the incremental rotation
        global_rotation = R.from_quat(global_quat)  # Convert global_quat to a Rotation object
        incremental_rotation = R.from_quat(delta_quat)  # Incremental rotation quaternion
        new_global_rotation = global_rotation * incremental_rotation  # Combine rotations
        
        # Step 5: Convert back to quaternion and Euler angles
        new_global_quat = new_global_rotation.as_quat()  # Updated quaternion (x, y, z, w)
        roll, pitch, yaw = new_global_rotation.as_rotvec( degrees=False)  # Roll, pitch, yaw
        
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
    
    def update_from_IMU_np_arr(self, angle_state: np.ndarray, depth_state: np.ndarray, dt=float):
        try:
            roll_rate = angle_state[0,1]
            pitch_rate = angle_state[1,1]
            yaw_rate = angle_state[2,1]
        except IndexError:
            raise ValueError("angle_state does not have the expected shape (3, 2)")

        try:
            depth_rate = depth_state[1]
        except IndexError:
            raise ValueError("depth_state does not have the expected shape (2,)")
    
        
        #self.orientation_estimate_quat, roll, pitch, yaw = self.update_orientation(self.orientation_estimate_quat, roll_rate, pitch_rate, yaw_rate, dt)

        roll=angle_state[0,0]
        pitch=angle_state[1,0]
        yaw=angle_state[2,0]

        #convert roll pitch yaw into quaternion

        self.orientation_estimate_quat = self.rotvec_to_quaternion(roll, pitch, yaw)

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

    def update_from_pres_np_arr(self, z, skip=False):

        self.position_estimate_vect[2] = z

        self.depth_ctrl.update_cdd(z)
        self.depth_ctrl.update_ddr(skip)
        


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
    """
    def update_loc(self, state): #unused
        #update loc data
        pass
    """

    def update_torques(self):
        # Update each PID based on current state and target state
        torque_x = self.roll_ctrl.update_dtau()
        torque_y = self.pitch_ctrl.update_dtau()
        torque_z = self.yaw_ctrl.update_dtau()
        return torque_x, torque_y, torque_z
    
    def update_thrust_z(self, skip=False):
        # Update depth PID based on current state and target state
        thrust_z = self.depth_ctrl.update_dt(skip)
        return thrust_z
    
    def check_orientation(self):
        # Check if the vehicle is oriented correctly by using orientation estimate quaternion

        # Step 1: Convert the orientation estimate quaternion to Euler angles
        roll, pitch, yaw = self.quaternion_to_rotvec(self.orientation_estimate_quat)
        roll_rate = self.roll_ctrl.current_detectable_angle_rate
        pitch_rate = self.pitch_ctrl.current_detectable_angle_rate
        #yaw_rate = self.yaw_ctrl.current_detectable_angle_rate

        # Step 2: Check if the vehicle is within the desired orientation margins
        roll_margin = np.abs(roll - self.roll_ctrl.desired_angle) < ANG_MARGIN
        pitch_margin = np.abs(pitch - self.pitch_ctrl.desired_angle) < ANG_MARGIN
        #yaw_margin = np.abs(yaw - self.yaw_ctrl.desired_angle) < ANG_MARGIN
        roll_rate_margin = np.abs(roll_rate - self.roll_ctrl.desired_angle_rate_local) < ANG_RATE_MARGIN
        pitch_rate_margin = np.abs(pitch_rate - self.pitch_ctrl.desired_angle_rate_local) < ANG_RATE_MARGIN
        #yaw_margin = ...

        if roll_margin and pitch_margin and roll_rate_margin and pitch_rate_margin:
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
        if target_state['yaw'] is not None and target_state['x'] is None and target_state['y'] is None:
            self.yaw_ctrl.update_da(target_state['yaw']) 
        if target_state['z'] is not None:
            self.depth_ctrl.update_dd(target_state['z'])

    def update_desired_arates(self):
        # Update desired angles and rates for each PID

        self.roll_ctrl.update_dar()
        self.pitch_ctrl.update_dar()
        self.yaw_ctrl.update_dar()

        self.convert_desired_arates_global_to_local()

    def convert_desired_arates_global_to_local(self):
        # Convert desired angle rates from global to local frame
        if not self.is_valid_quaternion(self.orientation_estimate_quat):
            raise ValueError("Invalid orientation estimate quaternion")
        if not self.is_valid_quaternion(self.orientation_estimate_quat):
            raise ValueError("Invalid orientation quaternion")
        # Get the desired angular rates in global frame
        desired_arates_global = np.array([
            self.roll_ctrl.desired_angle_rate_global,
            self.pitch_ctrl.desired_angle_rate_global,
            self.yaw_ctrl.desired_angle_rate_global
        ])

        # Compute the rotation matrix from body (local) frame to global frame
        R_body_to_global = R.from_quat(self.orientation_estimate_quat).as_matrix()

        # Transform desired angular rates from global frame to local (body) frame
        desired_arates_local = R_body_to_global.T @ desired_arates_global

        # Update desired angular rates in the local frame
        self.roll_ctrl.desired_angle_rate_local = desired_arates_local[0]
        self.pitch_ctrl.desired_angle_rate_local = desired_arates_local[1]
        self.yaw_ctrl.desired_angle_rate_local = desired_arates_local[2]
    def is_valid_quaternion(self, quat: np.ndarray):
        """
        Checks if a quaternion is valid (i.e., has unit norm).

        Args:
            quat (np.ndarray): Quaternion (x, y, z, w).

        Returns:
            bool: True if the quaternion is valid, False otherwise.
        """
        return np.isclose(np.linalg.norm(quat), 1.0)

    def is_valid_quaternion(self, quat: np.ndarray):
        """
        Checks if a quaternion is valid (i.e., has unit norm).

        Args:
            quat (np.ndarray): Quaternion (x, y, z, w).

        Returns:
            bool: True if valid, False otherwise.
        """
        return np.isclose(np.linalg.norm(quat), 1.0)

    def update_desired_drate(self, skip=False):
        # Update desired angles and rates for each PID
        self.depth_ctrl.update_ddr(skip)

