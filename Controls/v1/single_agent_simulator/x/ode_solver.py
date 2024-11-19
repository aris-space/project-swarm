import numpy as np


def euler(vehicle_model, y, u_prev, u_new, sim_freq):

    dt = 1/sim_freq

    return y + vehicle_model(y, u_new) * dt

def rk4(vehicle_model, y, u_prev, u_new, sim_freq):

    dt = 1/sim_freq

    # u_prev = u_t = u_t + 0.5*dt
    # u_new = u_t + dt

    #k are 2d vectors

    k1 = vehicle_model(y, u_prev)
    k2 = vehicle_model(y + 0.5 * dt * k1, u_prev)
    k3 = vehicle_model(y + 0.5 * dt * k2, u_prev)
    k4 = vehicle_model(y + dt * k3, u_new)
    return y + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)


    """
    Perform a single step of the RK4 method for a spring damper system => use this later on
    Args:
        f: The function defining the derivatives.
        y: The current state [x, x_dot].
        t: Current time.
        dt: Time step.
        u: Function for control input u(t).
    Returns:
        Updated state [x, x_dot] after one RK4 step.
    """