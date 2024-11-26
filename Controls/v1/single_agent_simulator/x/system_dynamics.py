import numpy as np
from utils.constants2 import *

def simple_system_dynamics(actual_state:np.ndarray, u:np.ndarray, a=0.0, b=0.0):
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
    x_dot_dot = (u + a * x + b * x_dot)

    return np.array([x_dot, x_dot_dot])

