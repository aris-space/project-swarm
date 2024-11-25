from __future__ import annotations

from single_agent_controller.controllers.depth_ctrl import depth_ctrl
from single_agent_controller.controllers.angle_ctrl import angle_ctrl
from single_agent_controller.controllers.pid_ctrl import PID
from single_agent_controller.controllers.pid_ctrl_w_error import PID_w_error
from utils.waypoints import *

from utils.constants2 import *

from scipy.spatial.transform import Rotation as R
import numpy as np


class LLC2:

    local_roll_ctrl: PID_w_error = None
    """controller for roll (rotation around local x-axis) => controls euler angles, due to the small changes in angle, they are deemed independant of each other"""  
    local_pitch_ctrl: PID_w_error = None
    """controller for pitch (rotation around local y-axis) => controls euler angles, due to the small changes in angle, they are deemed independant of each other"""
    local_yaw_ctrl: PID_w_error = None
    """controller for yaw (rotation around local z-axis) => controls euler angles, due to the small changes in angle, they are deemed independant of each other"""
    local_roll_rate_ctrl: PID = None
    """controller for roll rate (angular rate around local x-axis) => controls euler rates, due to the small changes in rate, they are deemed independant of each other"""
    local_pitch_rate_ctrl: PID = None
    """controller for pitch rate (angular rate around local y-axis) => controls euler rates, due to the small changes in rate, they are deemed independant of each other"""
    local_y_rate_ctrl: PID = None
    """controller for yaw rate (angular rate around local z-axis) => controls euler rates, due to the small changes in rate, they are deemed independant of each other"""
    global_orientation_estimate_quat: np.ndarray
    """global orientation estimate quaternion => scalar last"""
    global_orientation_target_quat: np.ndarray
    """global orientation target quaternion => scalar last"""
    desired_local_roll_rate: float
    desired_local_pitch_rate: float
    desired_local_yaw_rate: float
    """desired local rates"""
    actual_local_roll_rate: float
    actual_local_pitch_rate: float
    actual_local_yaw_rate: float
    """actual local yaw rate => should soon be replaced by class of system"""
    local_x_torque: float
    local_y_torque: float
    local_z_torque: float
    """local torques => should soon be replaced by class of system"""
    dt: float
    """controller sampling time"""
    

    def __init__(self, pid_params:dict, init_params:dict):
        """
        Initialize six PIDs, one for each angle and one for each rate in the local frame
        Initialize the global orientation estimate quaternion based on the initial euler yaw, pitch, roll
        Initialize the global orientation target quaternion based on the first waypoint
        Initialize the desired local angular rates to zero
        Initialize the actual local angular rates based on the inital euler rates yaw rate, pitch rate, roll rate
        Initialize the local torques to zero
        Initialize the controller sampling time to global T_LLC
        """

        self.local_roll_ctrl = PID_w_error(**pid_params['roll']['abs'])
        self.local_pitch_ctrl = PID_w_error(**pid_params['pitch']['abs'])
        self.local_yaw_ctrl = PID_w_error(**pid_params['yaw']['abs'])

        self.local_roll_rate_ctrl = PID(**pid_params['roll']['rate'])
        self.local_pitch_rate_ctrl = PID(**pid_params['pitch']['rate'])
        self.local_yaw_rate_ctrl = PID(**pid_params['yaw']['rate'])
        
        self.global_orientation_estimate_quat = self.euler_zyx_to_quaternion(init_params['yaw_init'], init_params['pitch_init'], init_params['roll_init'])
        self.global_orientation_target_quat = self.euler_zyx_to_quaternion(waypoints[0]['yaw'], waypoints[0]['pitch'], waypoints[0]['roll'])

        self.actual_local_roll_rate = init_params['roll_rate_init']
        self.actual_local_pitch_rate = init_params['pitch_rate_init']
        self.actual_local_yaw_rate = init_params['yaw_rate_init']

        self.desired_local_roll_rate = 0.0
        self.desired_local_pitch_rate = 0.0
        self.desired_local_yaw_rate = 0.0

        self.local_x_torque = 0.0
        self.local_y_torque = 0.0
        self.local_z_torque = 0.0

        self.dt = T_LLC

    def update_global_orientation_w_state(self, z,y,x, dt=1/LLC_FREQ):
        """
        update the global orientation estimate quaternion based on the euler angles yaw, pitch, roll
        => due to the small changes in angle per step, they are deemed independant of each other (they behave linearly)
        should be replaced when the system class is implemented
        """

        self.global_orientation_estimate_quat = self.euler_zyx_to_quaternion(z,y,x)
        return self.global_orientation_estimate_quat
    

    def update_global_orientation_w_dead_reckoning(self, yaw_rate, pitch_rate, roll_rate, dt=1/LLC_FREQ):
        """
        update the global orientation estimate quaternion based on the euler rates yaw rate, pitch rate, roll rate
        by performing a quaternion multiplication with the global_orientation_estimate_quat and the quaternion obtained by the euler rates
        """

        delta_euler_yaw, delta_euler_pitch, delta_euler_roll = yaw_rate*dt, pitch_rate*dt, roll_rate*dt

        q_delta = self.euler_zyx_to_quaternion(delta_euler_yaw, delta_euler_pitch, delta_euler_roll)
        q_delta = self.normalize_quaternion(q_delta)

        q_new = self.quaternion_multiply(q_delta, self.global_orientation_estimate_quat.copy())
        q_new = self.normalize_quaternion(q_new)
    
        self.global_orientation_estimate_quat = q_new    
        return q_new


    def calculate_error_quaternion(self):
        """
        calculate the error quaternion by multiplying the target quaternion with the conjugate of the current quaternion
        """

        q_target = self.global_orientation_target_quat.copy()
        q_current = self.global_orientation_estimate_quat.copy()
        q_conj = self.quaternion_conjugate(q_current)

        q_error = self.quaternion_multiply(q_target, q_conj)
        return q_error
        
    
    def calculate_local_angle_error(self):
        """
        calculate the local angle error by taking the first three elements of the error quaternion and multiplying them by 2
        """
        q_error = self.calculate_error_quaternion()
        local_error = 2 * q_error[0:3]
        return local_error
    
    def update_angle_pids(self, dt=1/LLC_FREQ):
        
        local_error = self.calculate_local_angle_error()
        self.desired_local_roll_rate = self.local_roll_ctrl.update(local_error[0], dt=1/LLC_FREQ)
        self.desired_local_pitch_rate = self.local_pitch_ctrl.update(local_error[1], dt=1/LLC_FREQ)
        self.desired_local_yaw_rate = self.local_yaw_ctrl.update(local_error[2], dt=1/LLC_FREQ)

    def update_angle_rate_pids(self,dt=1/LLC_FREQ):

        self.local_x_torque = self.local_roll_rate_ctrl.update(self.desired_local_roll_rate, self.actual_local_roll_rate, dt=1/LLC_FREQ)
        self.local_y_torque = self.local_pitch_rate_ctrl.update(self.desired_local_pitch_rate, self.actual_local_pitch_rate, dt=1/LLC_FREQ)
        self.local_z_torque = self.local_yaw_rate_ctrl.update(self.desired_local_yaw_rate, self.actual_local_yaw_rate, dt=1/LLC_FREQ)

        return self.local_x_torque, self.local_y_torque, self.local_z_torque

    def update_actual_local_rates(self, local_roll_rate, local_pitch_rate, local_yaw_rate):

        self.actual_local_roll_rate = local_roll_rate
        self.actual_local_pitch_rate = local_pitch_rate
        self.actual_local_yaw_rate = local_yaw_rate

    #from now on, the following functions are helper functions
    
    def quaternion_conjugate(self, q):
        """
        compute the conjugate of a quaternion
        """

        q_conj = q.copy()
        q_conj *= -1
        q_conj[3] *= -1
        return q_conj
    
    def normalize_quaternion(self, q):
        """
        normalize a quaternion
        """

        norm = np.linalg.norm(q)
        if norm == 0:
            return np.array([0.0, 0.0, 0.0, 1.0])  # Identity quaternion
        else:
            return q / norm
        
    def euler_zyx_to_quaternion(self, yaw, pitch, roll):
        """
        convert euler angles in to quaternion in the order yaw, pitch, roll,
        meaning that pitch is the rotation around the rotated y-axis => y' and roll is the rotation around the rotated x-axis => x''
        """

        return R.from_euler('zyx', [yaw, pitch, roll], degrees=False).as_quat()
    
    def quaternion_multiply(self, q1, q2):
        """
        multiply two quaternions
        """

        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2

        return np.array([x, y, z, w])
    
    def quaternion_to_euler_zyx(self, q):
        """
        convert a quaternion to euler angles in the order yaw, pitch, roll,
        meaning that pitch is the rotation around the rotated y-axis => y' and roll is the rotation around the rotated x-axis => x''
        """

        r = R.from_quat(q.copy())
        return r.as_euler('zyx', degrees=False)
    

    """
    Konzept:
    - initialisierung des globalen quaternions durch initial euler yaw pitch roll 'ZYX' => done
    - initialiserung des globalen ziel quaternions durch euler yaw pitch roll 'ZYX' => done
    - berechnung des error quaternions q_error = q_target * q_current.conj => done
    - konvertierung des q_error in local angular error = 2 * [q1,q2,q3] => done
    - q1 q2 q3 in local pid's einfügen, gibt w_x, w_y, w_z => done
    - w_x w_y w_z in local pid's einfügen, gibt tau_x, tau_y, tau_z => done
    - die tatsächlichen local angular rates zurückerhalten, in ersten pid einfügen => done
    - das neue globale quaternion erhalten mit dead reckoning  => done
    - berechnung, der benötigten winkelgeschwindigkeiten im globalen frame => done
    """