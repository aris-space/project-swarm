import numpy as np

def rk4 (t_s, h_s, x, f, vehicle_model):
    """
    arguments: t_s == time array of all times, h_s == time step size, 
    x == array of initial conditions and empty for each timestep , f == RHS of DE, (i.e. equations from our 6dof model) 
    returns: t_s array of all times, x == filled out conditions for all times 
    """

    for i in range(1,len(t_s)):
        k1 = f(t_s[i-1], x[:,i-1], vehicle_model)
        k2 = f(t_s[i-1]+h_s/2, x[:,i-1] + h_s*k1/2, vehicle_model)
        k3 = f(t_s[i-1]+h_s/2, x[:,i-1] + h_s*k2/2, vehicle_model)
        k4 = f(t_s[i-1]+h_s, x[:,i-1] + h_s*k3, vehicle_model)

        x[:,i] = x[:,i-1] + (h_s/6)*(k1+2*k2+2*k3+k4)

    return t_s, x