from __future__ import annotations

#from single_agent_controller.controllers.depth_ctrl import depth_ctrl
from single_agent_controller.controllers.pid_ctrl import PID
from single_agent_controller.controllers.pid_ctrl_w_error import PID_w_error
from utils.waypoints import *

#from v1.single_agent_controller.controllers.ca_system import SCA

from utils.constants2 import *

from scipy.spatial.transform import Rotation as R
import numpy as np


class LLC2:

    local_roll_ctrl: PID_w_error
    """controller for roll (rotation around local x-axis) => controls euler angles, due to the small changes in angle, they are deemed independant of each other"""  
    local_pitch_ctrl: PID_w_error
    """controller for pitch (rotation around local y-axis) => controls euler angles, due to the small changes in angle, they are deemed independant of each other"""
    local_yaw_ctrl: PID_w_error
    """controller for yaw (rotation around local z-axis) => controls euler angles, due to the small changes in angle, they are deemed independant of each other"""
    
    local_roll_rate_ctrl: PID
    """controller for roll rate (angular rate around local x-axis) => controls euler rates, due to the small changes in rate, they are deemed independant of each other"""
    local_pitch_rate_ctrl: PID
    """controller for pitch rate (angular rate around local y-axis) => controls euler rates, due to the small changes in rate, they are deemed independant of each other"""
    local_yaw_rate_ctrl: PID
    """controller for yaw rate (angular rate around local z-axis) => controls euler rates, due to the small changes in rate, they are deemed independant of each other"""

    global_x_ctrl: PID
    """controller for x position => controls position in global x direction"""
    global_y_ctrl: PID
    """controller for y position => controls position in global y direction"""
    global_z_ctrl: PID
    """controller for z position => controls position in global z (depth) direction"""

    local_r_ctrl: PID_w_error
    """controller for radial direction => controls distance in local r direction"""

    local_x_vel_ctrl: PID
    """controller for x velocity => controls velocity in local x direction"""
    local_y_vel_ctrl: PID
    """controller for y velocity => controls velocity in local y direction"""
    local_z_vel_ctrl: PID
    """controller for z velocity => controls velocity in local z (depth) direction"""

    global_orientation_estimate_quat: np.ndarray
    """global orientation estimate quaternion => scalar last"""
    global_orientation_target_quat: np.ndarray
    """global orientation target quaternion => scalar last"""

    global_position_estimate: np.ndarray
    """global position estimate"""
    global_position_target: np.ndarray
    """global position target"""

    desired_local_r: float
    """desired local r"""

    actual_local_roll_rate: float
    actual_local_pitch_rate: float
    actual_local_yaw_rate: float
    """actual local rates"""

    actual_local_x_vel: float
    actual_local_y_vel: float
    actual_local_z_vel: float
    """actual local velocities"""

    desired_local_roll_rate: float
    desired_local_pitch_rate: float
    desired_local_yaw_rate: float
    """desired local rates"""

    desired_global_x_vel: float
    desired_global_y_vel: float
    desired_global_z_vel: float
    """desired global velocities"""

    desired_local_x_vel: float
    desired_local_y_vel: float
    desired_local_z_vel: float
    """desired local velocities"""

    local_x_torque: float
    local_y_torque: float
    local_z_torque: float
    """local torques"""

    local_x_thrust: float
    local_y_thrust: float
    local_z_thrust: float
    """local thrusts"""

    dt: float
    """controller sampling time"""
    

    def __init__(self, pid_params:dict, init_params:dict):
        """
        Initialize six PIDs, one for each angle and one for each rate in the local frame
        Initialize two PIDs, one for x position and one for x velocity in the local frame
        Initialize two PIDs, one for z position and one for z velocity in the local frame
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

        self.global_x_ctrl = PID(**pid_params['x']['abs'])
        self.global_y_ctrl = PID(**pid_params['y']['abs'])
        self.global_z_ctrl = PID(**pid_params['z']['abs'])

        self.local_r_ctrl = PID_w_error(**pid_params['x']['abs'])  #(**pid_params['r']['abs']) change this later

        self.local_x_vel_ctrl = PID(**pid_params['x']['vel'])
        self.local_y_vel_ctrl = PID(**pid_params['y']['vel'])
        self.local_z_vel_ctrl = PID(**pid_params['z']['vel'])
        
        self.global_orientation_estimate_quat = self.euler_zyx_to_quaternion(init_params['yaw_init'], init_params['pitch_init'], init_params['roll_init'])
        self.global_orientation_target_quat = self.euler_zyx_to_quaternion(waypoints[0]['yaw'], waypoints[0]['pitch'], waypoints[0]['roll'])

        self.global_position_estimate = np.array([init_params['x_init'], init_params['y_init'], init_params['z_init']])
        self.global_position_target = np.array([waypoints[0]['x'], waypoints[0]['y'], waypoints[0]['z']])

        self.desired_local_r = 0.0

        self.actual_local_roll_rate = init_params['roll_rate_init']
        self.actual_local_pitch_rate = init_params['pitch_rate_init']
        self.actual_local_yaw_rate = init_params['yaw_rate_init']

        self.actual_local_x_vel = init_params['x_vel_init']
        self.actual_local_y_vel = init_params['y_vel_init']
        self.actual_local_z_vel = init_params['z_vel_init']

        self.desired_local_roll_rate = 0.0
        self.desired_local_pitch_rate = 0.0
        self.desired_local_yaw_rate = 0.0

        self.desired_global_x_vel = 0.0
        self.desired_global_y_vel = 0.0
        self.desired_global_z_vel = 0.0

        self.desired_local_x_vel = 0.0
        self.desired_local_y_vel = 0.0
        self.desired_local_z_vel = 0.0

        self.local_x_torque = 0.0
        self.local_y_torque = 0.0
        self.local_z_torque = 0.0

        self.local_x_thrust = 0.0
        self.local_y_thrust = 0.0
        self.local_y_thrust = 0.0

        self.dt = T_LLC


    def update_w_mode1(self):
        """
        mode 1: set a custom orientation, sub tries to go into that orientation => note: this does does not make the sub counteract buoyancy/gravity and currents
        """
        
        #update angle PIDs
        self.update_angle_pids()
        #update angle rate PIDs
        return self.update_angle_rate_pids()


    def update_w_mode2(self):
        """
        mode 2: set a position x,y,z, and the sub will try to reach it by just using x & z thrust => first sub goes into the correct roll & pitch angles (=0) and the correct yaw angle, so that global z thrust is equal to local z thrust.
        The desired yaw is calculated as the direction needed for reaching the specified position. As soon as the sub is within the angle margin, it will start to move towards the desired position
        """

        #translate planner to controller
        self.convert_global_x_y_to_local_r_yaw()

        #update angle & pos PIDs
        self.update_angle_pids()
        self.update_r_pids()

        #update angle rate and vel PIDs
        torquex, torquey, torquez = self.update_angle_rate_pids()
        thrustx, thrustz = self.update_r_vel_pids()

        return thrustx, 0, thrustz, torquex, torquey, torquez

    
    def update_w_mode3(self):
        """
        mode 3: set a position x,y,z, and an orientation yaw, pitch, roll, and the sub will move to the desired position and turn into the correct orientation simultaneously
        """

        #update local angle PIDs
        self.update_angle_pids()
        #update global position PIDs
        self.update_position_pids()

        #convert global velocities to local velocities
        self.convert_global_vel_to_local_vel()

        #update local angle rate PIDs
        torquex, torquey, torquez = self.update_angle_rate_pids()

        #update local velocity PIDs
        thrustx, thrusty, thrustz = self.update_veloctiy_pids()

        return thrustx, thrusty, thrustz, torquex, torquey, torquez


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
    
    def update_global_position(self, x, y, z):
        """
        update the global position estimate
        """

        self.global_position_estimate = np.array([x,y,z])
        return self.global_position_estimate


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
        """
        lets the local angle pid's run to calculate to desired local angular rates
        """
        
        local_error = self.calculate_local_angle_error()
        self.desired_local_roll_rate = self.local_roll_ctrl.update(local_error[0], dt=1/LLC_FREQ)
        self.desired_local_pitch_rate = self.local_pitch_ctrl.update(local_error[1], dt=1/LLC_FREQ)
        self.desired_local_yaw_rate = self.local_yaw_ctrl.update(local_error[2], dt=1/LLC_FREQ)


    def update_position_pids(self, dt=1/LLC_FREQ):
        """
        lets the global position pid's run to calculate to desired global velocities. Note that there still has to be a conversion into  local verlocities, which the local
        velocity pid's take as inputs
        """

        self.desired_global_x_vel = self.global_x_ctrl.update(self.global_position_target[0], self.global_position_estimate[0], dt=1/LLC_FREQ)
        self.desired_global_y_vel = self.global_y_ctrl.update(self.global_position_target[1], self.global_position_estimate[1], dt=1/LLC_FREQ)
        self.desired_global_z_vel = self.global_z_ctrl.update(self.global_position_target[2], self.global_position_estimate[2], dt=1/LLC_FREQ)


    def update_angle_rate_pids(self, dt=1/LLC_FREQ):
        """
        lets the local anglular rate pid's run to calculate desired local torques.
        """

        self.local_x_torque = self.local_roll_rate_ctrl.update(self.desired_local_roll_rate, self.actual_local_roll_rate, dt=1/LLC_FREQ)
        self.local_y_torque = self.local_pitch_rate_ctrl.update(self.desired_local_pitch_rate, self.actual_local_pitch_rate, dt=1/LLC_FREQ)
        self.local_z_torque = self.local_yaw_rate_ctrl.update(self.desired_local_yaw_rate, self.actual_local_yaw_rate, dt=1/LLC_FREQ)

        return self.local_x_torque, self.local_y_torque, self.local_z_torque


    def update_veloctiy_pids(self, dt=1/LLC_FREQ):
        """
        lets the local velocity pid's run to calculate to desired local thrusts.
        """

        self.local_x_thrust = self.local_x_vel_ctrl.update(self.desired_local_x_vel, self.actual_local_x_vel, dt=1/LLC_FREQ)
        self.local_y_thrust = self.local_y_vel_ctrl.update(self.desired_local_y_vel, self.actual_local_y_vel, dt=1/LLC_FREQ)
        self.local_z_thrust = self.local_z_vel_ctrl.update(self.desired_local_z_vel, self.actual_local_z_vel, dt=1/LLC_FREQ)

        return self.local_x_thrust, self.local_y_thrust, self.local_z_thrust


    def update_actual_local_rates(self, local_roll_rate, local_pitch_rate, local_yaw_rate):
        """
        stores local rates given by a SIM or IMU
        """

        self.actual_local_roll_rate = local_roll_rate
        self.actual_local_pitch_rate = local_pitch_rate
        self.actual_local_yaw_rate = local_yaw_rate


    def update_actual_local_velocities(self, local_x_vel, local_y_vel, local_z_vel):
        """
        stores local velocities given by a SIM or IMU
        """
            
        self.actual_local_x_vel = local_x_vel
        self.actual_local_y_vel = local_y_vel
        self.actual_local_z_vel = local_z_vel


    def update_r_pids(self, dt=1/LLC_FREQ):
        """
        for mode 2: checks if orientation is alright; then updates the r distance pid to get desired x velocity (r direction aligns with x direction if check orientation function is true)
        """

        if self.check_orientation_for_r_ctrl():
            self.desired_local_x_vel = self.local_r_ctrl.update(self.desired_local_r, skip=False)
            self.desired_local_z_vel = self.global_z_ctrl.update(self.global_position_target[2], self.global_position_estimate[2], skip=False)
        else:
            self.desired_local_x_vel = self.local_r_ctrl.update(self.desired_local_r, skip=True)
            self.desired_local_z_vel = self.global_z_ctrl.update(self.global_position_target[2], self.global_position_estimate[2], skip=True)


    def update_r_vel_pids(self, dt=1/LLC_FREQ):
        """
        checks if orientation is alright; then updates the x velocity pid to get desired x thrust
        """

        if self.check_orientation_for_r_ctrl():
            self.local_x_thrust = self.local_x_vel_ctrl.update(self.desired_local_x_vel, self.actual_local_x_vel, skip=False)
            self.local_z_torque = self.local_z_vel_ctrl.update(self.desired_local_z_vel, self.actual_local_z_vel, skip=False)
        else:
            self.local_x_thrust = self.local_x_vel_ctrl.update(self.desired_local_x_vel, self.actual_local_x_vel, skip=True)
            self.local_z_torque = self.local_z_vel_ctrl.update(self.desired_local_z_vel, self.actual_local_z_vel, skip=True)

        return self.local_x_thrust, self.local_z_torque
    
    # def check_orientation_for_z_ctrl(self):
    #     """
    #     check if the pitch and roll orientation is within a certain threshold of the target orientation and the rates close to 0
    #     """

    #     euler_x, euler_y, _ = self.quaternion_to_euler_zyx(self.global_orientation_estimate_quat)
    #     euler_x_target, euler_y_target, _ = self.quaternion_to_euler_zyx(self.global_orientation_target_quat)

    #     return np.abs(euler_x - euler_x_target) < ANG_MARGIN and np.abs(euler_y - euler_y_target) < ANG_MARGIN and np.abs(self.actual_local_roll_rate) < ANG_RATE_MARGIN and np.abs(self.actual_local_pitch_rate) < ANG_RATE_MARGIN
    
        
    def check_orientation_for_r_ctrl(self):
        """
        check if the pitch, roll and yaw orientation is within a certain threshold of the target orientation and the rates close to 0
        """

        euler_x, euler_y, euler_z = self.quaternion_to_euler_zyx(self.global_orientation_estimate_quat)
        euler_x_target, euler_y_target, euler_z_target = self.quaternion_to_euler_zyx(self.global_orientation_target_quat)

        return np.abs(euler_x - euler_x_target) < ANG_MARGIN and np.abs(euler_y - euler_y_target) < ANG_MARGIN and np.abs(euler_z - euler_z_target) < ANG_MARGIN and np.abs(self.actual_local_roll_rate) < ANG_RATE_MARGIN and np.abs(self.actual_local_pitch_rate) < ANG_RATE_MARGIN and np.abs(self.actual_local_yaw_rate) < ANG_RATE_MARGIN


    def convert_global_x_y_to_local_r_yaw(self):
        """
        Todo: calculate delta x and y, then convert to polar coordinates (x,yaw))
        """

        delta_x = self.global_position_target[0] - self.global_position_estimate[0]
        delta_y = self.global_position_target[1] - self.global_position_estimate[1]

        local_r = np.sqrt(delta_x**2 + delta_y**2)
        yaw = np.arctan2(delta_y, delta_x)

        if np.abs(local_r) < LANDING_ZONE_RADIUS :
            local_r = 0.0
            yaw, _, _ = self.quaternion_to_euler_zyx(self.global_orientation_estimate_quat)

        else:
            self.global_orientation_target_quat = self.euler_zyx_to_quaternion(yaw, 0, 0) #change this maybe? exchange 0's or pack into a function
            
        self.desired_local_r = local_r

    def convert_global_vel_to_local_vel(self):
        """
        convert the global x, y, z target to local x, y, z target
        """

        Rot_mat = R.from_quat(self.global_orientation_estimate_quat).as_matrix()

        #invert rotation matrix
        Rot_mat_inverse = np.linalg.inv(Rot_mat)

        self.desired_local_x_vel, self.desired_local_y_vel, self.desired_local_z_vel = Rot_mat_inverse @ np.array([self.desired_global_x_vel, self.desired_global_y_vel, self.desired_global_z_vel])

        return self.desired_local_x_vel, self.desired_local_y_vel, self.desired_local_z_vel

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