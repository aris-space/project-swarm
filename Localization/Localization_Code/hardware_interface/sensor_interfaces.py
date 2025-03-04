# This code imports Sensor data 
# At first, we will work with mock data to check whether the code works and then later on work with real-life data
 
import numpy as np 
from utilities_and_common_services.constants import speed_of_sound, atmospheric_pressure, water_density, gravity


def generate_pressure_data(positions):
    """
    Simulate pressure data based on depth (z-coordinate).
    :param positions: List of [x, y, z] positions.
    :return: Simulated pressure values.
    """
    depth = -np.array([pos[2] for pos in positions])  # Depth is negative z
    pressure_noise = 0.05  # meters
    pressure_data = depth + np.random.normal(0, pressure_noise, len(depth))
    return pressure_data

def generate_imu_yaw_data(yaw_angles, noise_std):
    """
    Generates IMU yaw data by adding Gaussian noise to the yaw angles.
    
    Parameters:
    - yaw_angles: List of true yaw angles.
    - noise_std: Standard deviation of the Gaussian noise.
    
    Returns:
    - imu_yaw_data: List of noisy yaw measurements.
    """
    imu_yaw_data = [yaw + np.random.normal(0, noise_std) for yaw in yaw_angles]
    return imu_yaw_data


def generate_robot_trajectory(total_time, dt, initial_position):
    """
    Generates a realistic robot trajectory over time.
    
    Parameters:
    - total_time: Total simulation time (seconds).
    - dt: Time step (seconds).
    - initial_position: Tuple (x0, y0, z0) representing the starting position.
    
    Returns:
    - positions: List of (x, y, z) positions over time.
    - time_steps: List of time stamps corresponding to each position.
    """
    time_steps = np.arange(0, total_time + dt, dt)
    positions = []
    yaw_angles = []
    pitch_angles = []
    roll_angles = []

    x, y, z = initial_position
    yaw, pitch, roll = 0.0, 0.0, 0.0  # Initial angles in radians

    # Append initial position
    positions.append((x, y, z))
    yaw_angles.append(yaw)
    pitch_angles.append(pitch)
    roll_angles.append(roll)

    for t in time_steps:
        if t <= 5.0:
            # Dive down for first 5 seconds
            dz = 1.0 * dt  # Assuming descent rate of 1 m/s
            z += dz
            dx = 0.0
            dy = 0.0
            yaw += 0.0  # No change in yaw
            pitch += 0.0  # No change in pitch
            roll += 0.0  # No change in roll
        elif t <= 10.0:
            # Move straight horizontally for next 5 seconds
            dz = 0.0
            dx = 1.0 * dt  # Assuming forward speed of 1 m/s
            dy = 0.0
            yaw += 0.0  # No change in yaw
            pitch += 0.0  # No change in pitch
            roll += 0.0  # No change in roll
        elif t <= 20.0:
            # Continue with a yaw rate of 0.05 rad/s
            dz = 0.0
            speed = 1.0  # Maintain speed of 1 m/s
            yaw_rate = 0.05  # Yaw rate in radians per second
            yaw += yaw_rate * dt  # Update yaw angle
            pitch += 0.0  # No change in pitch
            roll += 0.0  # No change in roll
            dx = speed * dt * np.cos(yaw)
            dy = speed * dt * np.sin(yaw)
        else:
            # move straight horizontally for the rest of the time
            speed = 1.0  # Maintain forward speed of 1 m/s
            dz = 0.0
            dx = speed * dt * np.cos(yaw)
            dy = speed * dt * np.sin(yaw)
            yaw += 0.0  # No change in yaw
            pitch += 0.0  # No change in pitch
            roll += 0.0  # No change in roll
    
    # update positions
        x += dx
        y += dy
        z += dz
        positions.append((x, y, z))
        yaw_angles.append(yaw)
        pitch_angles.append(pitch) 
        roll_angles.append(roll)
    
    return positions, yaw_angles, pitch_angles, roll_angles, time_steps


def simulate_hydrophone_measurements_at_positions(positions, yaw_angles, beacon_position, hydrophone_positions_local, total_time, speed_of_sound):
    """
    Simulates time measurements at hydrophones based on robot positions and orientations.

    Parameters:
    - positions: List of robot positions over time [(x1, y1, z1), (x2, y2, z2), ...].
    - yaw_angles: List of yaw angles over time [yaw1, yaw2, ...].
    - beacon_position: Tuple (x_beacon, y_beacon, z_beacon).
    - hydrophone_positions_local: List of hydrophone positions in the robot's local frame [(x_h1, y_h1, z_h1), ...].
    - speed_of_sound: Speed of sound in water (meters per second).

    Returns:
    - time_measurements: List of time measurements for each hydrophone at each time step [[t1_h1, t1_h2, ...], [t2_h1, t2_h2, ...], ...].
    """
    time_measurements = []
    for pos, yaw in zip(positions, yaw_angles):
        x_robot, y_robot, z_robot = pos

        # Transform hydrophone positions to global frame
        hydrophones_global = []
        for h_local in hydrophone_positions_local:
            x_h_local, y_h_local, z_h_local = h_local
            # Rotate hydrophone positions based on yaw
            x_h_global = x_robot + (x_h_local * np.cos(yaw) - y_h_local * np.sin(yaw))
            y_h_global = y_robot + (x_h_local * np.sin(yaw) + y_h_local * np.cos(yaw))
            z_h_global = z_robot + z_h_local  # Assuming no pitch or roll
            hydrophones_global.append((x_h_global, y_h_global, z_h_global))

        # Calculate distances and time measurements
        t_measurements = []
        for h_global in hydrophones_global:
            # Calculate distance between hydrophone and beacon
            distance = np.linalg.norm(np.array(h_global) - np.array(beacon_position))
            # Calculate time measurement
            t = distance / speed_of_sound
            t_measurements.append(t)
        time_measurements.append(t_measurements)
    for t_measurements in time_measurements:
        for t in t_measurements:
            if t < 0 or t > total_time:
                print(f"Invalid time measurement: {t}")
    return time_measurements


def compute_velocities(positions, dt):
    """
    Computes velocities from positions.
    
    Parameters:
    - positions: List of (x, y, z) positions over time.
    - dt: Time step (seconds).
    
    Returns:
    - velocities: List of (v_x, v_y, v_z) velocities over time.
    """
    velocities = []
    for i in range(len(positions)):
        if i == 0:
            # Assume initial velocity is zero
            velocities.append((0.0, 0.0, 0.0))
        else:
            x_prev, y_prev, z_prev = positions[i - 1]
            x_curr, y_curr, z_curr = positions[i]
            v_x = (x_curr - x_prev) / dt
            v_y = (y_curr - y_prev) / dt
            v_z = (z_curr - z_prev) / dt
            velocities.append((v_x, v_y, v_z))
    return velocities

def compute_accelerations(velocities, dt):
    """
    Computes accelerations from velocities.
    
    Parameters:
    - velocities: List of (v_x, v_y, v_z) velocities over time.
    - dt: Time step (seconds).
    
    Returns:
    - accelerations: List of (a_x, a_y, a_z) accelerations over time.
    """
    accelerations = []
    for i in range(len(velocities)):
        if i == 0:
            # Assume initial acceleration is zero
            accelerations.append((0.0, 0.0, 0.0))
        else:
            v_x_prev, v_y_prev, v_z_prev = velocities[i - 1]
            v_x_curr, v_y_curr, v_z_curr = velocities[i]
            a_x = (v_x_curr - v_x_prev) / dt
            a_y = (v_y_curr - v_y_prev) / dt
            a_z = (v_z_curr - v_z_prev) / dt
            accelerations.append((a_x, a_y, a_z))
    return accelerations

def simulate_pressure_data(positions):
    """
    Simulates pressure data based on depth (z-coordinate) from trajectory data.
    """
    pressures = []
    for x, y, z in positions:
        pressure = atmospheric_pressure + water_density * gravity * z
        pressures.append(pressure)
    return pressures 