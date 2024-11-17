import math
import numpy as np

def state_equations(t, x, vehicle_model):
    """t == time, x == Vector of current states, vehicle_model == dict of vehicle properties
    
    variables are named under following convention <name>_<frame>_<units>

    t in s
    x:
    x[0] = u (velocity in x)
    x[1] = v (velocity in y)
    x[2] = w (velocity in z)
    x[3] = p (angular velocity around x) (roll rate)
    x[4] = q (angular velocity around y) (pitch rate)
    x[5] = r (angular velocity around z) (yaw rate)
    x[6] = Thrust in x
    x[7] = Thrust in y
    x[8] = Thrust in z
    x[9] = Torque in x
    x[10] = Torque in y
    x[11] = Torque in z
    
    Returns dx"""

    dx = np.empty(6)

    #current velocities
    u_b_mps = x[0]
    v_b_mps = x[1]
    w_b_mps = x[2]

    #current angular speeds
    p_b_rps = x[3]
    q_b_rps = x[4]
    r_b_rps = x[5]

    #Vehicle model
    m_kg = vehicle_model["weight"]
    Jxx_b_kgm2 = vehicle_model["MoI Tensor"][0,0]
    Jyy_b_kgm2 = vehicle_model["MoI Tensor"][1,1]
    Jzz_b_kgm2 = vehicle_model["MoI Tensor"][2,2]

    #External Forces
    Fx_b_kgmps = 1
    Fy_b_kgmps = 1
    Fz_b_kgmps = 0

    #External Moments
    L__b_kgm2ps2 = 0
    M__b_kgm2ps2 = 0
    N__b_kgm2ps2 = 0

    #Equations of motion
    #Translational equations
    # du_b_mps2
    dx[0] = 1/m_kg*Fx_b_kgmps - w_b_mps*q_b_rps + v_b_mps*r_b_rps
    # dv_b_mps2
    dx[1] = 1/m_kg*Fy_b_kgmps - u_b_mps*r_b_rps + w_b_mps*p_b_rps
    # dw_b_mps2
    dx[2] = 1/m_kg*Fz_b_kgmps - v_b_mps*p_b_rps + u_b_mps*q_b_rps

    #Rotational equations | Moments: M=[L,M,N]T | w=[p,q,r]T angular speed (radiants/second)  
    #Jxx, Jyy,Jzz = ...
    #inv_J_b = 1/J_b...
    
    # dp_b_rps2
    dx[3] = (L__b_kgm2ps2 + r_b_rps*q_b_rps*(Jyy_b_kgm2 - Jzz_b_kgm2))/Jxx_b_kgm2 #(-(Jzz_b_kgm2*(Jzz_b_kgm2-Jyy_b_kgm2))*q_b_rps*r_b_rps + Jzz_b_kgm2*L_b_kgm2ps2) / (Jxx_b_kgm2*Jzz_b_kgm2) 
    # dq_b_rps2
    dx[4] = (M__b_kgm2ps2 + p_b_rps*r_b_rps*(Jzz_b_kgm2 - Jxx_b_kgm2))/Jyy_b_kgm2 #(Jzz_b_kgm2-Jxx_b_kgm2)*r_b_rps*p_b_rps + M_b_kgm2ps2) / Jyy_b_kgm2
    # dr_b_rps2
    dx[5] = (N__b_kgm2ps2 + p_b_rps*q_b_rps*(Jyy_b_kgm2 - Jxx_b_kgm2))/Jzz_b_kgm2 #((Jxx_b_kgm2*(Jxx_b_kgm2-Jyy_b_kgm2))*q_b_rps*p_b_rps + Jxx_b_kgm2*N_b_kgm2ps2) / (Jxx_b_kgm2*Jzz_b_kgm2)

    #Body orientation (angles) CS (Coordinate system) wrt NED (North East Down) CS (inertial)

    #Euler angular in NED rates <=> Body angle rates in body frame
    
    return dx