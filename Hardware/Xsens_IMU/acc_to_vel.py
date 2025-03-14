def comp_vel_drift(old_comp, position_delta_pos_system, position_delta_vel_int):
    return comp_vel

def convert_acc_to_vel (comp_vel, new_acc, quat, dt):

    #new_acc = np.array([ax, ay, az])
    #quat = np.array([qw, qx, qy, qz])
    
    if (quat(0) < 0) {
      quat = -1.0*quat
    }

    // AHRS Transformations
    C_N2B = quat2dcm(quat);
    C_B2N = C_N2B.transpose();

    // obtain euler angles from quaternion
    std::tie(phi, theta, psi) = toEulerAngles(quat);

    // Velocity Update
    dx = C_B2N*new_acc + grav;
    vn_ins += _dt*dx(0,0); #v_north
    ve_ins += _dt*dx(1,0); #v_east
    vd_ins += _dt*dx(2,0); #v_down

    current_vel = comp_vel + np.array([vn_ins, ve_ins, vd_ins]);

    return current_vel