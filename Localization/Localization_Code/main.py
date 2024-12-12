import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from utilities_and_common_services.agents import Robot
from utilities_and_common_services.constants import speed_of_sound
from hardware_interface.sensor_interfaces import generate_imu_yaw_data, generate_robot_trajectory, simulate_hydrophone_measurements_at_positions, compute_accelerations, compute_velocities, simulate_pressure_data
from localization_algorithms.algorithms import calculate_distance, select_closest_hydrophones, trilateration_2D, least_squares_optimization, f, h, compute_F, compute_H
from utilities_and_common_services.coordinate_systems import calc_robot_global_position, transform_to_global, transform_to_local

# define operational area (e.g., 100 x 100 meters)
area_width = 100
area_height = 100
area_depth = 30

# define beacon position (origin of global frame)
# for simplicity of the simulation, it is implemented as 0,0 atm but this can easily be adjusted later on
x_beacon = 0
y_beacon = 0
z_beacon = 0
beacon_position = (x_beacon, y_beacon, z_beacon)

# define initial position of the robot in the global frame
x_robot_initial = 2
y_robot_initial = 0
z_robot_initial = 0 # meters

# define hydrophone positions in the robot's local frame
hydrophone_positions_local = [(0.2, -0.1, 0.0), (-0.2, -0.1, 0.0), (-0.2, 0.1, 0.0), (0.2, 0.1, 0.0)]
# define distances between hydrophones
# order: d_12, d_13, d_14, d_23, d_24, d_34
distances_between_hydrophones = (0.4, np.sqrt(0.2), 0.2, 0.2, np.sqrt(0.2), 0.4)

# total simulation time
total_time = 30 # seconds
# time step for smooth trajectory generation 
dt = 0.5

# time steps for position calculation
position_dt = 0.5

# generate trajectory, positions & yaw angles: global frame
initial_position = (x_robot_initial, y_robot_initial, z_robot_initial)
positions, yaw_angles, time_steps = generate_robot_trajectory(total_time, dt, initial_position)

# compute velocities and accelerations in the global frame
# -> since they are derived from positions, they are in the global frame
# this makes sense since velocities shouldn't be in the local frame since it then would always be 0 (local frame moves with the robot)
velocities = compute_velocities(positions, dt)
accelerations = compute_accelerations(velocities, dt)

# indices for position calculation times
position_time_steps = np.arange(0, total_time + position_dt, position_dt)
position_indices = [int(t / dt) for t in position_time_steps]

# extract positions and yaw angles at position calculation intervals
positions_calc = [positions[i] for i in position_indices]
yaw_angles_calc = [yaw_angles[i] for i in position_indices]
pitch_angles_calc = [0] * len(positions_calc)
roll_angles_calc = [0] * len(positions_calc)

# initial state:
vx_0, vy_0, vz_0 = velocities[0]
theta_0 = yaw_angles_calc[0]
state_estimate = np.array([x_robot_initial, y_robot_initial, z_robot_initial, vx_0, vy_0, vz_0, theta_0])
# state: x,y,z,vx,vy,vz,theta: global frame

# simulate time measurements at the calculated positions
time_measurements = simulate_hydrophone_measurements_at_positions(positions_calc, yaw_angles_calc, beacon_position, hydrophone_positions_local, total_time, speed_of_sound)

# convert time measurements to distances
distances = [calculate_distance(toa_measurements, speed_of_sound) for toa_measurements in time_measurements]


# initial state covariance matrix (initial uncertainty)
P = np.diag([
    1.0**2,  # Variance in position x (e.g., standard deviation of 0.5 meters)
    1.0**2,  # Variance in position y
    1.0**2,  # Variance in position z
    0.5**2,  # Variance in velocity vx (e.g., standard deviation of 0.1 m/s)
    0.5**2,  # Variance in velocity vy
    0.5**2,  # Variance in velocity vz
    np.deg2rad(10)**2  # Variance in yaw angle (e.g., 5 degrees in radians)
])

# Process noise covariance Q
# Tune these values based on expected process noise
sigma_acc = 0.5 # Acceleration noise standard deviation (m/s^2)
sigma_gyro = np.deg2rad(2) # Gyroscope noise standard deviation (rad/s)
Q = np.diag([
    0.25 * dt**4 * sigma_acc**2,  # Position x
    0.25 * dt**4 * sigma_acc**2,  # Position y
    0.25 * dt**4 * sigma_acc**2,  # Position z
    dt**2 * sigma_acc**2,         # Velocity x
    dt**2 * sigma_acc**2,         # Velocity y
    dt**2 * sigma_acc**2,         # Velocity z
    dt**2 * sigma_gyro**2         # Yaw angle
])

# Measurement noise covariance R
# Tune these values based on sensor characteristics
sigma_distance = 0.001  # ToA distance measurement noise (meters)
sigma_yaw = 0.01       # IMU yaw measurement noise (radians)
sigma_pressure = 0.005  # Pressure sensor noise (meters)
R = np.diag([sigma_distance**2] * len(hydrophone_positions_local) + [sigma_yaw**2, sigma_pressure**2])

# initialize lists to store estimated positions 
estimated_positions = []
estimated_velocities = []
estimated_orientations = []

# EKF loop implementation
for k, idx in enumerate(position_indices):
    # current imu data
    imu_acceleration = accelerations[idx]  # (a_x, a_y, a_z) in local frame
    imu_yaw_measurement = yaw_angles_calc[k]          # No noise added (noise_std=0.0)

    # Compute angular rate (omega_z) from IMU yaw data
    if k == 0:
        omega_z = 0.0  # No previous data to compute rate
    else:
        omega_z = (yaw_angles_calc[k] - yaw_angles_calc[k - 1]) / position_dt

    # Prediction Step
    state_predict = f(state_estimate, imu_acceleration, omega_z, position_dt)
    F_k = compute_F(state_estimate, imu_acceleration, omega_z, position_dt)
    P_predict = F_k @ P @ F_k.T + Q

    # Measurement Update Step
    # Retrieve measurements
    toa_distances = distances[k]  # List of distances to each hydrophone¨
    depth = simulate_pressure_data(positions)
    z_actual = np.array(toa_distances + [imu_yaw_measurement, depth[idx]])  # Include yaw measurement and depth
    
    # Predicted measurements
    z_predict = h(state_predict, hydrophone_positions_local, beacon_position)
    z_predict = np.array(z_predict)  # Include predicted yaw angle and pressure depth
    
    # Measurement residual
    y = z_actual - z_predict
    
    # Compute Jacobian H_k
    H_k = compute_H(state_predict, hydrophone_positions_local, beacon_position)
    #H_k_extended = np.vstack([H_k, np.array([0, 0, 0, 0, 0, 0, 1])])  # Add row for yaw measurement
    
    # Innovation covariance
    S_k = H_k @ P_predict @ H_k.T + R

    # Add a small epsilon to the diagonal of S_k to prevent numerical issues
    epsilon = 1e-6
    S_k += epsilon * np.eye(S_k.shape[0])
    
    # Kalman Gain
    K_k = P_predict @ H_k.T @ np.linalg.inv(S_k)
    
    # Update state estimate
    state_estimate = state_predict + K_k @ y
    
    # Update error covariance
    P = (np.eye(len(state_estimate)) - K_k @ H_k) @ P_predict
    
    # Store estimates
    estimated_positions.append(state_estimate[:3].tolist())
    estimated_velocities.append(state_estimate[3:6].tolist())
    estimated_orientations.append(state_estimate[6])

# convert list to numpy array
estimated_positions = np.array(estimated_positions)
true_positions = np.array(positions_calc)

"""
# loop through the calculated positions and estimate the robot's global position through trilateration and least squares optimization
for i in range(len(positions_calc)):
    # Current measurements and IMU data
    toa_measurements = time_measurements[i]
    imu_yaw_current = imu_yaw[i]
    theta_robot = imu_yaw_current  # Current yaw angle

    # local acceleration data
    accelerations_local = transform_to_local(accelerations, theta_robot)

    # robot and beacon depths
    z_robot = positions_calc[i][2]
    z_beacon_local = z_beacon - z_robot

    # select the three closest hydrophones based on projected distances
    selected_positions, selected_projected_distances = select_closest_hydrophones(toa_measurements, hydrophone_positions_local, z_beacon, z_robot, speed_of_sound)

    # perform trilateration using the selected hydrophones
    beacon_local_position_2D = trilateration_2D(selected_positions, selected_projected_distances)
    beacon_local_position_3D = (beacon_local_position_2D[0], beacon_local_position_2D[1], z_beacon_local)
    trilateration_robot_position = calc_robot_global_position(x_beacon, y_beacon, z_beacon, beacon_local_position_3D[0], beacon_local_position_3D[1], beacon_local_position_3D[2], theta_robot)

    # calculate distance for the current time step
    distances_current = calculate_distance(toa_measurements, speed_of_sound)
    # check for None before proceeding
    if distances_current is None:
        print(f"Distance calculation failed at time step {i}.")
        estimated_positions.append((np.nan, np.nan, np.nan))
        continue

    # estimate the robot's global position
    estimated_robot_position = least_squares_optimization(hydrophone_positions_local, distances_current, beacon_position, theta_robot, initial_guess=trilateration_robot_position)

    if estimated_robot_position:
        x_est, y_est, z_est = estimated_robot_position
        estimated_positions.append((x_est, y_est, z_est))
    else:
        # If estimation failed, append NaNs or handle appropriately
        print ("Estimation failed at time step", position_time_steps[i])
        estimated_positions.append((np.nan, np.nan, np.nan))
    """

errors = []
# Compare estimated positions with actual positions
for i, t in enumerate(position_time_steps):
    x_actual, y_actual, z_actual = positions_calc[i]
    x_est, y_est, z_est = estimated_positions[i]
    
    if not np.isnan(x_est):
        error = np.sqrt((x_actual - x_est)**2 + (y_actual - y_est)**2 + (z_actual - z_est)**2)
        errors.append(error)
        print(f"Time {t:.1f}s: Error = {error:.2f} meters")
    else:
        print(f"Time {t:.1f}s: Estimation failed.")

print(f"The largest error is: {max(errors):.2f} meters")
# Store sensor data in the robot
# robot.store_sensor_data('pressure', pressure_data)
# robot.store_sensor_data('imu_yaw', imu_yaw)
# robot.store_sensor_data('imu_pitch', imu_pitch)
# robot.store_sensor_data('imu_roll', imu_roll)

# Print the stored sensor data for verification
# print(f"Calculated Depth: {depths}")
# print(f"IMU Yaw Data: {robot.get_sensor_data('imu_yaw')}")
# print(f"IMU Pitch Data: {robot.get_sensor_data('imu_pitch')}")
# print(f"IMU Roll Data: {robot.get_sensor_data('imu_roll')}")
# print(f"Simulated Robot Position: {x_robot_actual, y_robot_actual, z_robot_actual}")
# print(f"Calculated Possible Robot Position 1: {robot_position[0]}")
# print(f"Calculated Possible Robot Position 2: {robot_position[1]}")
# print(f"Calculated Robot Position 1 in Global Frame: {robot_position_global}")
# print(f"Calculated Robot Position 2 in Global Frame: {robot_position2_global}")
# print(f"Error: {error}")

# print(f"Estimated Robot Position: {estimated_positions}")

# print the initial state
print(f"True Initial State: {x_robot_initial, y_robot_initial, z_robot_initial, vx_0, vy_0, vz_0, theta_0}")
print(f"Estimated Initial State: {estimated_positions[0][0], estimated_positions[0][1], estimated_positions[0][2], estimated_velocities[0][0], estimated_velocities[0][1], estimated_velocities[0][2], estimated_orientations[0]}")


# plot:
# Extract coordinates
x_actual = [pos[0] for pos in positions_calc]
y_actual = [pos[1] for pos in positions_calc]
z_actual = [pos[2] for pos in positions_calc]

x_estimated = [pos[0] for pos in estimated_positions]
y_estimated = [pos[1] for pos in estimated_positions]
z_estimated = [pos[2] for pos in estimated_positions]

# Filter out NaN values from estimated positions
x_estimated_filtered = []
y_estimated_filtered = []
z_estimated_filtered = []

for x, y, z in zip(x_estimated, y_estimated, z_estimated):
    if not np.isnan(x) and not np.isnan(y) and not np.isnan(z):
        x_estimated_filtered.append(x)
        y_estimated_filtered.append(y)
        z_estimated_filtered.append(z)

# Create a 3D plot
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot actual trajectory
ax.plot(x_actual, y_actual, z_actual, 'b-', label='Actual Trajectory')

# Plot estimated trajectory
ax.plot(x_estimated_filtered, y_estimated_filtered, z_estimated_filtered, 'r--', label='Estimated Trajectory')

# Label axes
ax.set_xlabel('X Position (m)', labelpad=10)
ax.set_ylabel('Y Position (m)', labelpad=10)
ax.set_zlabel('Depth (m)', labelpad=10)

# Set title
ax.set_title('Actual vs. Estimated Trajectory (3D Plot)')

# Since depth increases with positive Z, no need to invert Z-axis
# Set Z-axis limits starting from 0
max_depth = max(max(z_actual), max(z_estimated_filtered)) if len(z_estimated_filtered) > 0 else max(z_actual)
ax.set_zlim(0, max_depth + 1)  # Add a margin if desired

# Add legend
ax.legend()

# Show grid
ax.grid(True)

# Show plot
plt.show()
