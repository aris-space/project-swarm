# This code imports Sensor data 
# At first, we will work with mock data to check whether the code works and then later on work with real-life data
 
import numpy as np 


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

def generate_imu_data(total_time, dt):
    """
    Simulate IMU yaw data based on a constant yaw rate with noise.
    :param total_time: Total time for the simulation (float).
    :param dt: Time step (float).
    :return: Simulated IMU yaw data.
    """
    time_steps = np.arange(0, total_time, dt)

    yaw_rate = 0.05  # Radians per second
    pitch_rate = 0.03
    roll_rate = 0.02

    yaw = yaw_rate * time_steps  # True yaw value
    pitch = pitch_rate * time_steps
    roll = roll_rate * time_steps

    noise_std = 0.005
    imu_yaw = yaw + np.random.normal(0, noise_std, len(time_steps))  # Single yaw measurement with noise
    imu_pitch = pitch + np.random.normal(0, noise_std, len(time_steps))
    imu_roll = roll + np.random.normal(0, noise_std, len(time_steps))
    
    return imu_yaw, imu_pitch, imu_roll

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
    time_steps = np.arange(0, total_time, dt)
        
    # Initialize positions list
    positions = [initial_position]
    
    # Define movement parameters
    speed = 1.0  # meters per second
    yaw_rate = 0.05  # radians per second (change in yaw per second)
    
    # Initial yaw angle
    yaw = 0.0
    
    for t in time_steps[1:]:
        # Update yaw angle
        yaw += yaw_rate * dt  # Assuming constant yaw rate
        
        # Compute displacement
        dx = speed * dt * np.cos(yaw)
        dy = speed * dt * np.sin(yaw)
        dz = 0.0  # Assuming constant depth
        
        # Update position
        x_new = positions[-1][0] + dx
        y_new = positions[-1][1] + dy
        z_new = positions[-1][2] + dz
        
        positions.append((x_new, y_new, z_new))
    
    return positions, time_steps
