from utilities_and_common_services.constants import speed_of_sound, gravity, water_density
import numpy as np
from scipy.optimize import minimize

# calculate z-coordinate from pressure data

def calculate_depth(pressure_data):
    """
    Calculate depth from pressure data.
    :param pressure_data: Pressure data.
    :return: Depth.
    """
    depths = []
    for pressure in pressure_data:
        # Calculate depth using the formula h = P / (rho * g)
        depth = pressure / (water_density * gravity)
        depths.append(float(depth))
    return depths 

# calculate distance from TDoA data
def calculate_distance(TDoA_data):
    """
    Calculate distance from TDoA data.
    :param TDoA_data: TDoA data.
    :return: Distance.
    """
    distances = []
    for tdoa in TDoA_data:
        # Calculate distance using the formula d = c * TDoA
        distance = speed_of_sound * tdoa
        distances.append(float(distance))
    return distances


def trilateration(t1, t2, h1_local, h2_local, z_robot_actual, z_beacon):
    """
    This function calculates the coordinates of the beacon in the robot's local frame based on two different 
    measurements at hydrophone 1 & hydrophone 2.
    """
    d1 = t1 * speed_of_sound
    d2 = t2 * speed_of_sound
    x_h1 = h1_local[0]
    y_h1 = h1_local[1]
    z_h1 = h1_local[2]
    x_h2 = h2_local[0]
    y_h2 = h2_local[1]
    z_h2 = h2_local[2]

    z_beacon_local = z_beacon - z_robot_actual        

    d_hydrophones = np.sqrt((x_h2 - x_h1)**2 + (y_h2 - y_h1)**2 + (z_h2 - z_h1)**2)

    # integrate depth information
    delta_z1 = z_beacon_local - z_h1 # depth difference between hydrophone 1 and beacon in the local frame
    delta_z2 = z_beacon_local - z_h2 # depth difference between hydrophone 2 and beacon in the local frame
    d1_projected = np.sqrt(d1**2 - delta_z1**2) # projected distance between hydrophone 1 and beacon
    d2_projected = np.sqrt(d2**2 - delta_z2**2) # projected distance between hydrophone 2 and beacon

    if abs(d1_projected-d2_projected) <= d_hydrophones <= (d1_projected+d2_projected):
        print("Circles intersect at two points")

    # calculate the midpoint between the hydrophones
    a = (d1_projected**2 - d2_projected**2 + d_hydrophones**2) / (2 * d_hydrophones)
    # calculate the height from the midpoint to the intersection point
    h = np.sqrt(d1_projected**2 - a**2)

    xm = x_h1 + a * (x_h2 - x_h1) / d_hydrophones
    ym = y_h1 + a * (y_h2 - y_h1) / d_hydrophones

    # find the intersection points
    x_intersection1 = xm + h * (y_h2 - y_h1) / d_hydrophones
    y_intersection1 = ym - h * (x_h2 - x_h1) / d_hydrophones
    x_intersection2 = xm - h * (y_h2 - y_h1) / d_hydrophones
    y_intersection2 = ym + h * (x_h2 - x_h1) / d_hydrophones
    
    return ([float(x_intersection1), float(y_intersection1), z_beacon_local],
            [float(x_intersection2), float(y_intersection2), z_beacon_local])

def calculate_bearing(x_local, y_local):
    """
    Calculates the bearing from the robot to the beacon in the robot's local frame.
    
    Parameters:
    - x_local, y_local: Beacon's position in the robot's local frame.
    
    Returns:
    - bearing: Bearing in the robot's local frame (radians).
    """
    bearing = np.arctan2(y_local, x_local)
    return bearing

