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

    dt: float
    """simulation times"""
    

    def __init__(self, pid_params:dict, init_params:dict):
        # Initialize six PIDs for each degree of freedom
        #self.depth_ctrl = depth_ctrl(pid_params['depth'], llc_freq)

        self.dt = T_LLC

        self.local_roll_ctrl = PID_w_error(**pid_params['roll']['abs'])
        self.local_pitch_ctrl = PID_w_error(**pid_params['pitch']['abs'])
        self.local_yaw_ctrl = PID_w_error(**pid_params['yaw']['abs'])

        self.local_roll_rate_ctrl = PID(**pid_params['roll']['rate'])
        self.local_pitch_rate_ctrl = PID(**pid_params['pitch']['rate'])
        self.local_yaw_rate_ctrl = PID(**pid_params['yaw']['rate'])
        
        self.global_orientation_estimate_quat = self.euler_zyx_to_quaternion(init_params['yaw_init'], init_params['pitch_init'], init_params['roll_init'])
        print("global orientation estimate quat: ", self.global_orientation_estimate_quat)
        print("global orientation estimate euler: ", self.quaternion_to_euler_zyx(self.global_orientation_estimate_quat))
        self.global_orientation_target_quat = self.euler_zyx_to_quaternion(waypoints[0]['yaw'], waypoints[0]['pitch'], waypoints[0]['roll'])

        self.desired_local_roll_rate = 0.0
        self.desired_local_pitch_rate = 0.0
        self.desired_local_yaw_rate = 0.0

        self.actual_local_roll_rate = 0.0
        self.actual_local_pitch_rate = 0.0
        self.actual_local_yaw_rate = 0.0

        self.local_x_torque = 0.0
        self.local_y_torque = 0.0
        self.local_z_torque = 0.0

    def update_global_orientation_estimate_quat_easy(self, z,y,x, dt=1/LLC_FREQ):
        #update using quaternions, where the scalar part is in the back with a rotvec, not a matrix
        self.global_orientation_estimate_quat = self.euler_zyx_to_quaternion(z,y,x)
        return self.global_orientation_estimate_quat
    
    def update_global_orientation_estimate_quat_hard(self, yaw_rate, pitch_rate, roll_rate, dt=1/LLC_FREQ):
        euler_yaw, euler_pitch, euler_roll = yaw_rate*dt, pitch_rate*dt, roll_rate*dt
        q_delta = self.euler_zyx_to_quaternion(euler_yaw, euler_pitch, euler_roll)
        q_delta = self.normalize_quaternion(q_delta)
        #q_new = self.quaternion_multiply(self.quaternion_conjugate(q_delta), self.quaternion_conjugate(self.global_orientation_estimate_quat))
        #q_new = self.normalize_quaternion(q_new)
        #self.global_orientation_estimate_quat = self.quaternion_conjugate(q_new)
        q_new = self.quaternion_multiply(q_delta, self.global_orientation_estimate_quat.copy())
        q_new = self.normalize_quaternion(q_new)
        self.global_orientation_estimate_quat = q_new    
        return q_new
    



    def calculate_error_quaternion(self):
        q_target = self.global_orientation_target_quat.copy()
        q_current = self.global_orientation_estimate_quat.copy()
        q_conj = self.quaternion_conjugate(q_current)
        q_error = self.quaternion_multiply(q_target, q_conj)
        return q_error
        
    
    def calculate_local_angle_error(self):
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
    
    def quaternion_conjugate(self, q):
        q_conj = q.copy()
        q_conj *= -1
        q_conj[3] *= -1
        return q_conj
    
    def normalize_quaternion(self, q):
        norm = np.linalg.norm(q)
        if norm == 0:
            return np.array([0.0, 0.0, 0.0, 1.0])  # Identity quaternion
        else:
            return q / norm
        
    def euler_zyx_to_quaternion(self, yaw, pitch, roll):
        return R.from_euler('zyx', [yaw, pitch, roll], degrees=False).as_quat()
    
    def quaternion_multiply(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        return np.array([x, y, z, w])
    
    def quaternion_to_euler_zyx(self, q):
        r = R.from_quat(q.copy())
        return r.as_euler('zyx', degrees=False)
    
    def from_rotvec_to_quaternion(self, rotvec):
        return R.from_rotvec(rotvec).as_quat()
        
    #initialisierung des globalen quaternions durch initial euler yaw pitch roll 'ZYX' => done
    #initialiserung des globalen ziel quaternions durch euler yaw pitch roll 'ZYX' => done
    #berechnung des error quaternions q_error = q_target * q_current.conj => done
    #konvertierung des q_error in local angular error = 2 * [q1,q2,q3] => done
    #q1 q2 q3 in local pid's einfügen, gibt w_x, w_y, w_z => done
    #w_x w_y w_z in local pid's einfügen, gibt tau_x, tau_y, tau_z => done
    #die tatsächlichen local angular rates zurückerhalten, in ersten pid einfügen => done
    #das neue globale quaternion erhalten mit q_new = q_current + dt/2 * Omega ('angular vel. matr.) * q current => done
    #berechnung, der benötigten winkelgeschwindigkeiten im globalen frame                   