import numpy as np

def simple_system_dynamics(actual_state, u, a=0.0, b=0.0):
    """
    Defines the system dynamics for x_dot_dot = u(t) + a*x + b*x_dot.
    Args:
        actual_state: actual state vector [x, x_dot].
        u: Control input u(t).
        a: Coefficient for x.
        b: Coefficient for x_dot.
    Returns:
        Derivative vector [x_dot, x_dot_dot].
    """
    x, x_dot = actual_state
    x_dot_dot = u + a * x + b * x_dot

    return np.array([x_dot, x_dot_dot])

import numpy as np

def complex_system_dynamics(state, control_inputs, params):
    """
    Compute the dynamics of an underwater drone with added mass forces and centrifugal forces.
    
    Parameters:
    - state: np.array([x, y, z, u, v, w, roll, pitch, yaw, p, q, r])
        Position (x, y, z) [m], linear velocities (u, v, w) [m/s],
        orientation (roll, pitch, yaw) [rad], and angular velocities (p, q, r) [rad/s].
    - control_inputs: np.array([Fx, Fy, Fz, Mx, My, Mz])
        Forces [N] and moments [Nm] in the body frame.
    - params: dict
        Contains physical and environmental parameters:
        - "mass": Drone mass [kg].
        - "dimensions": (a, b, c) dimensions of the cuboid [m].
        - "rho": Fluid density [kg/m^3].
        - "Cd": Drag coefficients (linear, angular).
        - "added_mass": Added mass coefficients for linear and angular motion.

    Returns:
    - derivatives: np.array([dx, dy, dz, du, dv, dw, droll, dpitch, dyaw, dp, dq, dr])
        Time derivatives of state variables.
    """
    # Extract parameters
    mass = params["mass"]
    a, b, c = params["dimensions"]
    rho = params["rho"]
    Cd_linear, Cd_angular = params["Cd"]
    added_mass = params["added_mass"]

    # State variables
    u, v, w = state[3:6]  # Linear velocities
    p, q, r = state[9:]   # Angular velocities

    # Control inputs
    Fx, Fy, Fz, Mx, My, Mz = control_inputs

    # Buoyancy and gravity
    g = 9.81  # Gravity [m/s^2]
    buoyancy_force = rho * g * a * b * c  # Archimedes' principle
    net_force_z = buoyancy_force - mass * g  # Net vertical force

    # Drag forces
    drag_force = -0.5 * rho * np.array([
        Cd_linear[0] * u * abs(u),
        Cd_linear[1] * v * abs(v),
        Cd_linear[2] * w * abs(w)
    ])
    drag_moment = -0.5 * rho * np.array([
        Cd_angular[0] * p * abs(p),
        Cd_angular[1] * q * abs(q),
        Cd_angular[2] * r * abs(r)
    ])

    # Added mass forces (linear and angular)
    linear_acceleration = np.array([u, v, w])
    angular_acceleration = np.array([p, q, r])
    added_mass_force = -np.array([
        added_mass[0] * u,
        added_mass[1] * v,
        added_mass[2] * w
    ])
    added_mass_moment = -np.array([
        added_mass[3] * p,
        added_mass[4] * q,
        added_mass[5] * r
    ])

    # Centrifugal forces (assuming the drone rotates and moves in space)
    # These are more relevant when the drone undergoes high angular velocities.
    centrifugal_force = -mass * np.cross(np.array([p, q, r]), np.cross(np.array([p, q, r]), np.array([u, v, w])))

    # Equations of motion
    linear_acc = (np.array([Fx, Fy, Fz + net_force_z]) + drag_force + added_mass_force) / mass
    angular_acc = (np.array([Mx, My, Mz]) + drag_moment + added_mass_moment) / (1.0 + added_mass[3:])

    # Time derivatives
    dx = np.zeros(12)
    dx[0:3] = state[3:6]  # Position derivatives
    dx[3:6] = linear_acc  # Linear acceleration
    dx[6:9] = state[9:]   # Orientation derivatives
    dx[9:] = angular_acc  # Angular acceleration

    return dx
