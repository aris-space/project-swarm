import numpy as np

def system_dynamics(y, u, a=0.0, b=0.0):
    """
    Defines the system dynamics for x_dot_dot = u(t) + a*x + b*x_dot.
    Args:
        y: State vector [x, x_dot].
        t: Current time (not used explicitly here but useful for non-autonomous systems).
        u: Control input u(t).
        a: Coefficient for x.
        b: Coefficient for x_dot.
    Returns:
        Derivative vector [x_dot, x_dot_dot].
    """
    x, x_dot = y
    x_dot_dot = u + a * x + b * x_dot
    return np.array([x_dot, x_dot_dot])

