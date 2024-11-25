import numpy as np

# this layer is is responsible for mathematcal operations and common functions

# calculate angular differences:
def angular_difference(angle1, angle2):
    """
    Calculates the smallest difference between two angles.

    Parameters:
    - angle1, angle2: Angles in radians.

    Returns:
    - difference: Smallest angular difference in radians.
    """
    return np.arctan2(np.sin(angle1 - angle2), np.cos(angle1 - angle2))