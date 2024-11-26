import math
import numpy as np

def state_equations(t_s: np.array, x: np.array, vehicle_model: dict):
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

    #current angular speeds
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

    #Position !!!NOT IN GLOBAL FRAME!!!
    dx[0] = 0 #x[6]
    dx[1] = 0 #x[7]
    dx[2] = 0 #x[8] 

    #Rotation !!!NOT IN GLOBAL FRAME!!!
    dx[3] = 0 #x[9]
    dx[4] = 0 #x[10]
    dx[5] = 0 #x[11]

    #Equations of motion
    #Translational equations
    # du_b_mps2
    dx[6] = 1/m_kg*Fx_b_kgmps - w_b_mps*q_b_rps + v_b_mps*r_b_rps
    # dv_b_mps2
    dx[7] = 1/m_kg*Fy_b_kgmps - u_b_mps*r_b_rps + w_b_mps*p_b_rps
    # dw_b_mps2
    dx[8] = 1/m_kg*Fz_b_kgmps - v_b_mps*p_b_rps + u_b_mps*q_b_rps

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