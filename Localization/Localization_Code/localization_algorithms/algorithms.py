from utilities_and_common_services.constants import speed_of_sound, gravity, water_density, atmospheric_pressure
import numpy as np
from scipy.optimize import least_squares

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
        depth = (pressure - atmospheric_pressure) / (water_density * gravity)
        depths.append(float(depth))
    return depths 

# calculate distance from time measurements
def calculate_distance(toa_measurements, speed_of_sound):
    """
    Calculate distance from ToA data.
    :param toa_measurements: ToA data.
    :return: Distance.
    """
    distances = []
    for t in toa_measurements:
        if t <= 0:
            print(f"Invalid TOA measurement: {t}")
            return None
        distances.append(t * speed_of_sound)
    return distances

def select_closest_hydrophones(time_measurements, hydrophone_positions_local, z_beacon, z_robot_actual, speed_of_sound):
    """
    Selects the three hydrophones with the smallest time measurements.

    Parameters:
    - toa_measurements: List of time-of-arrival measurements [t1, t2, ..., tN]
    - hydrophone_positions: List of hydrophone positions [(x1, y1, z1), ..., (xN, yN, zN)]
    - speed_of_sound: Speed of sound in the medium (m/s)

    Returns:
    - selected_positions: List of positions [(x1, y1), (x2, y2), (x3, y3)] of the selected hydrophones
    - selected_projected_distances: List of projected distances [d1_proj, d2_proj, d3_proj]
    """
    # calculate delta z (same for all hydrophones since they are on the same depth as the center of the robot)
    delta_z = z_robot_actual - z_beacon

    # Calculate distances from TOA measurements
    distances = [t * speed_of_sound for t in time_measurements]

    # calculate projected distances in the x-y plane
    projected_distances = []
    for i, d in enumerate(distances):
        expression = d**2 - delta_z**2
        print(f"Hydrophone {i}: d = {d}, delta_z = {delta_z}, d^2 - delta_z^2 = {expression}")
        if expression >= 0:
            d_proj = np.sqrt(expression)
            projected_distances.append(d_proj)
        else:
            print(f"Warning: Negative value under square root for hydrophone {i}.")
            projected_distances.append(float('inf'))  # Exclude this hydrophone

    # Pair distances with hydrophone indices
    distance_hydrophone_pairs = list(zip(projected_distances, hydrophone_positions_local))

    # Sort pairs by distance (smallest to largest)
    sorted_pairs = sorted(distance_hydrophone_pairs, key=lambda x: x[0])

    # Select the three closest hydrophones
    selected_pairs = sorted_pairs[:3]

    # Extract selected positions and distances
    selected_projected_distances = [pair[0] for pair in selected_pairs]
    selected_positions = [(pair[1][0], pair[1][1]) for pair in selected_pairs]

    return selected_positions, selected_projected_distances

def trilateration_2D(hydrophone_positions_2D, projected_distances):
    """
    First implementation: This function calculates the coordinates of the beacon in the robot's local frame based on two different 
    measurements at hydrophone 1 & hydrophone 2.
    
    Since we want to scale up to more hydrophones, the function needs to be adjusted. The current idea is to use time measurements
    of the first three hydrophones to calculate the beacon's position in the robot's local frame.
    """
    if len(hydrophone_positions_2D) != 3 or len(projected_distances) != 3:
        print("Exactly three hydrophones are required.")
        return None

    # Extract coordinates
    x1, y1 = hydrophone_positions_2D[0]
    x2, y2 = hydrophone_positions_2D[1]
    x3, y3 = hydrophone_positions_2D[2]

    # Extract distances
    r1, r2, r3 = projected_distances

    # Set up the equations based on the circle equations
    # Equation between circle 1 and circle 2
    A = 2 * (x2 - x1)
    B = 2 * (y2 - y1)
    D = r1**2 - r2**2 - x1**2 + x2**2 - y1**2 + y2**2

    # Equation between circle 1 and circle 3
    E = 2 * (x3 - x1)
    F = 2 * (y3 - y1)
    G = r1**2 - r3**2 - x1**2 + x3**2 - y1**2 + y3**2

    # Solve for x and y
    denominator = A * F - B * E
    if abs(denominator) < 1e-10:
        print("Denominator too small; circles may not intersect properly.")
        return None

    x = (D * F - B * G) / denominator
    y = (A * G - D * E) / denominator

    return x, y

def least_squares_optimization(hydrophone_positions_local, distances, beacon_position, theta_robot, initial_guess):
    """
    Estimates the beacon position using nonlinear least squares multilateration.
    
    Parameters:
    - hydrophone_positions: List of tuples [(x1, y1, z1), (x2, y2, z2), ..., (xN, yN, zN)]
    - distances: List of distances [d1, d2, ..., dN]

    Compute Residuals: 
    - Instead of finding exact intersection points, calculate how close a candidate position is to satisfying each measurement.
    - Residuals are the differences between the estimated distances and the actual distances.
    - The goal is to minimize the sum of squared residuals.
    - quantify how well the estimated position explains the measured distances.
    
    Returns:
    - estimated_position: Tuple (x, y, z) or None if optimization fails
    """
    def residuals(position, hydrophones_local, distances, beacon_pos, theta):
        x_r, y_r, z_r = position
        res = []
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        # Transform hydrophone position to global frame
        for (x_h_local, y_h_local, z_h_local), d in zip(hydrophones_local, distances):
            x_h_global = x_r + x_h_local * cos_theta - y_h_local * sin_theta
            y_h_global = y_r + x_h_local * sin_theta + y_h_local * cos_theta
            z_h_global = z_r + z_h_local  # Assuming no rotation around x and y axes
            
            # Calculate distance from beacon to hydrophone
            d_calc = np.sqrt(
                (beacon_pos[0] - x_h_global)**2 +
                (beacon_pos[1] - y_h_global)**2 +
                (beacon_pos[2] - z_h_global)**2
            )
            res.append(d_calc - d)
        
        return res
    
    # Initial guess for the robot's position (could be the last known position or (0,0,0))
    if initial_guess is None:
        initial_guess = [0.0, 0.0, 0.0]
    
    # Perform least squares optimization
    result = least_squares(
        residuals,
        initial_guess,
        args=(hydrophone_positions_local, distances, beacon_position, theta_robot)
    )
    
    if result.success:
        return tuple(result.x) # return estimated position
    else:
        print("Optimization failed.")
        return None

# Kalman filter
# process model:
# purpose: predicts the next state of the system given the current state and control inputs 
def f(state, imu_acceleration, imu_angular_rates, dt):
    x, y, z, vx, vy, vz, yaw, pitch, roll = state
    # Extract IMU data
    ax, ay, az = imu_acceleration  
    omega_x, omega_y, omega_z = imu_angular_rates

    # Update positions
    x_new = x + vx * dt + 0.5 * ax * dt**2
    y_new = y + vy * dt + 0.5 * ay * dt**2
    z_new = z + vz * dt + 0.5 * az * dt**2
   
    # Update velocities
    v_x_new = vx + ax * dt
    v_y_new = vy + ay * dt
    v_z_new = vz + az * dt
    
    # Update orientation
    yaw_new = yaw + omega_z * dt
    pitch_new = pitch + omega_y * dt
    roll_new = roll + omega_x * dt
    
    return np.array([x_new, y_new, z_new, v_x_new, v_y_new, v_z_new, yaw_new, pitch_new, roll_new])

# state transition matrix:
def compute_F(state, imu_acceleration, imu_angular_rate, dt):
    # Jacobian of f with respect to state
        F = np.eye(7)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        # Velocity derivatives are zero (assuming acceleration is input)
        # Yaw derivative is zero
        return F

# measurement model:
# purpose: predicts the expected sensor measurements based on the current state estimate
def h(state, hydrophone_positions_local, beacon_position):
    x, y, z, vx, vy, vz, theta = state
    
    # Calculate distances from hydrophones to the beacon
    distances = []
    for x_h, y_h, z_h in hydrophone_positions_local:
        # Transform hydrophone position to global frame
        x_h_global = x + x_h * np.cos(theta) - y_h * np.sin(theta)
        y_h_global = y + x_h * np.sin(theta) + y_h * np.cos(theta)
        z_h_global = z + z_h  # Assuming no rotation around x and y axes
        
        # Calculate distance from beacon to hydrophone
        d = np.sqrt(
            (beacon_position[0] - x_h_global)**2 +
            (beacon_position[1] - y_h_global)**2 +
            (beacon_position[2] - z_h_global)**2
        )
        distances.append(d)

    # predicted pressure
    pressure_predicted = atmospheric_pressure + water_density * gravity * z
    # combine measurements
    measurements = distances + [theta] + [pressure_predicted]
    return measurements

# Jacobian of the state transition function f
# Jacobian matrix: matrix of all first-order partial derivatives of a vector-valued function (linear approximation of the function)
def compute_H(state, hydrophone_positions_local, beacon_position):
    x, y, z, vx, vy, vz, theta = state # global
    N = len(hydrophone_positions_local)
    H = np.zeros((N + 2, 7))  # Adjusted for state vector length of 7
    
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    for i, (x_h, y_h, z_h) in enumerate(hydrophone_positions_local):
        # Transform hydrophone position to global frame
        x_h_global = x + x_h * cos_theta - y_h * sin_theta
        y_h_global = y + x_h * sin_theta + y_h * cos_theta
        z_h_global = z + z_h

        delta_x = beacon_position[0] - x_h_global
        delta_y = beacon_position[1] - y_h_global
        delta_z = beacon_position[2] - z_h_global
        d = np.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

        # Partial derivatives w.r.t x, y, z
        H[i, 0] = -delta_x / d  # Negative sign due to measurement residual calculation
        H[i, 1] = -delta_y / d
        H[i, 2] = -delta_z / d

        # Partial derivative w.r.t theta
        dx_dtheta = -x_h * sin_theta - y_h * cos_theta
        dy_dtheta = x_h * cos_theta - y_h * sin_theta
        H[i, 6] = (delta_x * dx_dtheta + delta_y * dy_dtheta) / d

        # Partial derivatives w.r.t velocities are zero
        # H[i, 3] to H[i, 5] remain zero
    H[N+1, 2] = water_density * gravity  # Partial derivative w.r.t z for pressure measurement

    return H


# once the functionality of the EKF is tested, it will be implemented as a class to create a clearer structure 
