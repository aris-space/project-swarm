import numpy as np

def posang(t_s, h_s, x, pos_ang):
    """Integrates all the velocities for each time step to get the position and angle"""
    for i in range(1,len(t_s)):
        pos_ang[:,i] = pos_ang[:,i-1] + x[:,i-1]*h_s

    return pos_ang


