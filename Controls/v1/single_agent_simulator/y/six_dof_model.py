import math
import numpy as np
from utils.constants2 import *

def state_equations(x: np.array, vehicle_model: dict):
    """Attributes:
    t_ms == time, x == Vector of current states, vehicle_model == dict of vehicle properties
    
    variables are named under following convention <name>_<frame>_<units>

    x:
    x[0] = x (pos) (left)
    x[1] = y (pos) 
    x[2] = z (pos) (down)
    x[3] = roll (rad)
    x[4] = pitch (rad)
    x[5] = yaw (rad)
    x[6] = u (velocity in x)
    x[7] = v (velocity in y)
    x[8] = w (velocity in z)
    x[9] = p (angular velocity around x) (roll rate)
    x[10] = q (angular velocity around y) (pitch rate)
    x[11] = r (angular velocity around z) (yaw rate)
    x[12] = Thrust in x
    x[13] = Thrust in y
    x[14] = Thrust in z
    x[15] = Torque in x
    x[16] = Torque in y
    x[17] = Torque in z
    
    Returns dx"""

    dx = np.empty(18)

    #current velocities
    u_b_mps = x[6]
    v_b_mps = x[7]
    w_b_mps = x[8]

    #current angular rates
    p_b_rps = x[9]
    q_b_rps = x[10]
    r_b_rps = x[11]
    
    #Vehicle model
    m_kg = vehicle_model["weight"]
    Jxx_b_kgm2 = vehicle_model["MoI Tensor"][0,0]
    Jyy_b_kgm2 = vehicle_model["MoI Tensor"][1,1]
    Jzz_b_kgm2 = vehicle_model["MoI Tensor"][2,2]

    #External Forces
    Fx_b_kgmps = x[12]
    Fy_b_kgmps = x[13]
    Fz_b_kgmps = x[14]

    #External Moments (here may still be something foul => need to be really small)
    L__b_kgm2ps2 = x[15]
    M__b_kgm2ps2 = x[16]
    N__b_kgm2ps2 = x[17]


    #get rotation matrix from euler angles
    #Roll
    phi = x[3]
    #Pitch
    theta = x[4]
    #Yaw
    psi = x[5]

    rot_mat = euler_to_rotation_matrix(phi, theta, psi)

    #convert local velocities to global velocities
    v_glob_mps = rot_mat @ np.array([u_b_mps, v_b_mps, w_b_mps])

    dx_glob = np.zeros(3)
    dx_glob[0] = v_glob_mps[0]
    dx_glob[1] = v_glob_mps[1]
    dx_glob[2] = v_glob_mps[2]


    # #Position !!!NOT IN GLOBAL FRAME!!!
    # dx[0] = x[6]
    # dx[1] = x[7]
    # dx[2] = x[8]

    #Position in global frame
    dx[0] = dx_glob[0]
    dx[1] = dx_glob[1]
    dx[2] = dx_glob[2]

    #Rotation !!!NOT IN GLOBAL FRAME!!!
    dx[3] = x[9] 
    dx[4] = x[10]
    dx[5] = x[11]

    #Equations of motion
    #Translational equations
    # du_b_mps2
    dx[6] = 1/m_kg*Fx_b_kgmps - u_b_mps*DRAG_COEFFICIENT - w_b_mps*q_b_rps + v_b_mps*r_b_rps #we need drag forces, otherwise system continues to move forever with 0 thrust
    # dv_b_mps2
    dx[7] = 1/m_kg*Fy_b_kgmps - v_b_mps*DRAG_COEFFICIENT - u_b_mps*r_b_rps + w_b_mps*p_b_rps #we need drag forces, otherwise system continues to move forever with 0 thrust
    # dw_b_mps2
    dx[8] = 1/m_kg*Fz_b_kgmps - w_b_mps*DRAG_COEFFICIENT - v_b_mps*p_b_rps + u_b_mps*q_b_rps #we need drag forces, otherwise system continues to move forever with 0 thrust

    #Rotational equations | Moments: M=[L,M,N]T | w=[p,q,r]T angular speed (radiants/second)  
    #Jxx, Jyy, Jzz = ...
    #inv_J_b = 1/J_b...
    
    # dp_b_rps2
    dx[9] = (L__b_kgm2ps2 + r_b_rps*q_b_rps*(Jyy_b_kgm2 - Jzz_b_kgm2))/Jxx_b_kgm2 #(-(Jzz_b_kgm2*(Jzz_b_kgm2-Jyy_b_kgm2))*q_b_rps*r_b_rps + Jzz_b_kgm2*L_b_kgm2ps2) / (Jxx_b_kgm2*Jzz_b_kgm2) 
    # dq_b_rps2
    dx[10] = (M__b_kgm2ps2 + p_b_rps*r_b_rps*(Jzz_b_kgm2 - Jxx_b_kgm2))/Jyy_b_kgm2 #(Jzz_b_kgm2-Jxx_b_kgm2)*r_b_rps*p_b_rps + M_b_kgm2ps2) / Jyy_b_kgm2
    # dr_b_rps2
    dx[11] = (N__b_kgm2ps2 + p_b_rps*q_b_rps*(Jyy_b_kgm2 - Jxx_b_kgm2))/Jzz_b_kgm2 #((Jxx_b_kgm2*(Jxx_b_kgm2-Jyy_b_kgm2))*q_b_rps*p_b_rps + Jxx_b_kgm2*N_b_kgm2ps2) / (Jxx_b_kgm2*Jzz_b_kgm2)

    #Body orientation (angles) CS (Coordinate system) wrt NED (North East Down) CS (inertial)

    #Euler angular in NED rates <=> Body angle rates in body frame

    #Do not set to higher than 0 or it explodes!
    dx[12] = 0#x[12]
    dx[13] = 0#x[13]
    dx[14] = 0#x[14]

    dx[15] = 0 #x[15]
    dx[16] = 0 #x[16]
    dx[17] = 0 #x[17]
    
    return dx


def euler_to_rotation_matrix(phi, theta, psi):
    """
    Converts Euler angles (phi, theta, psi) in z-y-x sequence to a rotation matrix.
    """
    Rz = np.array([
        [math.cos(psi), -math.sin(psi), 0],
        [math.sin(psi), math.cos(psi), 0],
        [0, 0, 1]
    ])

    Ry = np.array([
        [math.cos(theta), 0, math.sin(theta)],
        [0, 1, 0],
        [-math.sin(theta), 0, math.cos(theta)]
    ])

    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(phi), -math.sin(phi)],
        [0, math.sin(phi), math.cos(phi)]
    ])

    # Combined rotation matrix
    R = Rz @ Ry @ Rx
    return R