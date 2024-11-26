import numpy as np
from utilities_and_common_services.agents import Robot
from utilities_and_common_services.constants import speed_of_sound
from hardware_interface.sensor_interfaces import generate_imu_data, generate_pressure_data
from localization_algorithms.algorithms import trilateration, calculate_bearing
from utilities_and_common_services.coordinate_systems import GlobalCoordinateSystem, LocalCoordinateSystem, calc_robot_global_position, transform_to_global, transform_bearing_to_global
from utilities_and_common_services.math_utils import angular_difference

# Define parameters for the single agent
# positions = np.random.uniform(-10, 10, (10, 3))  # Simulated trajectory
total_time = 10  # seconds
dt = 0.1  # time step

# define operational area (e.g., 100 x 100 meters)
area_width = 100
area_height = 100
area_depth = 30

# define beacon position (origin of global frame)
# for simplicity of the simulation, it is implemented as 0,0 atm but this can easily be adjusted later on
x_beacon = 0
y_beacon = 0
z_beacon = 0

# generate the actual position of the robot in the global frame 
# For simulation purposes, we need this information to create realisitic time measurements
# However, in the estimation process, the robot will not use this information
x_robot_actual = np.random.uniform(0, area_width)
y_robot_actual = np.random.uniform(0, area_height)
z_robot_actual = np.random.uniform(0, area_depth)
positions = x_robot_actual, y_robot_actual, z_robot_actual

# Create a single robot instance
robot = Robot(name="TestRobot", positions=positions)

# Generate sensor data
# pressure_data = generate_pressure_data(positions)
# imu_yaw, imu_pitch, imu_roll = generate_imu_data(total_time, dt)
# initially, the robot should be oriented in the same direction as the global frame
imu_yaw_initial = 0 
imu_yaw = 0.3
initial_bearing = np.deg2rad(45)

# define hydrophone positions in the local frame
d_hydrophones = 0.4
h1_local = np.array([0.0, 0.0, -0.2])
h2_local = np.array([d_hydrophones, 0.0, -0.2])

# transform hydrophone positions to the global frame for simulation purposes
h1_global = transform_to_global(h1_local[0], h1_local[1], h1_local[2], x_robot_actual, y_robot_actual, z_robot_actual, imu_yaw)
h2_global = transform_to_global(h2_local[0], h2_local[1], h2_local[2], x_robot_actual, y_robot_actual, z_robot_actual, imu_yaw)

# calculate true distances from hydrophones to beacon (for simulation)
# this is kind of redundant atm since beacon position is 0,0 but this way the code is more flexible in case we want to test different beacon positions
d1_true = np.linalg.norm(h1_global - np.array([x_beacon, y_beacon, z_beacon]))
d2_true = np.linalg.norm(h2_global - np.array([x_beacon, y_beacon, z_beacon]))

# simulated time measurements
t1_true = d1_true / speed_of_sound
t2_true = d2_true / speed_of_sound

# calculate the possible positions of the beacon in the robot's local frame
beacon_local_positions = trilateration(t1_true, t2_true, h1_local, h2_local, z_robot_actual, z_beacon)
robot_position1_global = calc_robot_global_position(x_beacon, y_beacon, z_beacon, beacon_local_positions[0][0], beacon_local_positions[0][1], beacon_local_positions[0][2], imu_yaw)
robot_position2_global = calc_robot_global_position(x_beacon, y_beacon, z_beacon, beacon_local_positions[1][0], beacon_local_positions[1][1], beacon_local_positions[1][2], imu_yaw)

# Next step: determine which of the two possible positions is the correct one 
# idea: use IMU yaw data to determine the robots' orientation. Initially, the robots' orientation should be identical to the beacon's.
# We always know the orientation of the robot and we know where the fixed beacon is initially positioned.
# This way, we should be able to determine which direction the acoustic signals should come from.

# calulate the bearings to possible beacon positions in the robot's local frame
bearing1_local = calculate_bearing(beacon_local_positions[0][0], beacon_local_positions[0][1])
bearing2_local = calculate_bearing(beacon_local_positions[1][0], beacon_local_positions[1][1])

bearing1_global = transform_bearing_to_global(bearing1_local, imu_yaw)
bearing2_global = transform_bearing_to_global(bearing2_local, imu_yaw)
# as the robot doesn't know its global position, but we have the imu data, we can estimate the bearing to the beacon in the local frame
# As the robot rotates, the direction to the beacon in the robot's local frame changes by the negative of the robot's yaw angle.
# if the beacon is initially straight ahead, the expected bearing should be -theta_robot at all times 
expected_bearing = initial_bearing -imu_yaw
# calculate the angular difference between the expected bearing and the bearings to the possible beacon positions
angular_diff1 = abs(angular_difference(bearing1_global, expected_bearing))
angular_diff2 = abs(angular_difference(bearing2_global, expected_bearing))
# selct the position with the smallest angular difference
if angular_diff1 < angular_diff2:
    selected_beacon_local = beacon_local_positions[0]
else:
    selected_beacon_local = beacon_local_positions[1]

# calculate the global position of the robot
robot_position_global = calc_robot_global_position(x_beacon, y_beacon, z_beacon, selected_beacon_local[0], selected_beacon_local[1], selected_beacon_local[2], imu_yaw) 

# estimate calculation errors
error = np.sqrt((robot_position_global[0] - x_robot_actual)**2 + (robot_position_global[1] - y_robot_actual)**2 + (robot_position_global[2] - z_robot_actual)**2)



# Store sensor data in the robot
# robot.store_sensor_data('pressure', pressure_data)
robot.store_sensor_data('imu_yaw', imu_yaw)
# robot.store_sensor_data('imu_pitch', imu_pitch)
# robot.store_sensor_data('imu_roll', imu_roll)
robot.store_sensor_data('time_1', t1_true)
robot.store_sensor_data('time_2', t2_true)


# Print the stored sensor data for verification
# print(f"Pressure Data: {robot.get_sensor_data('pressure')}")
# print(f"Calculated Depth: {depths}")
# print(f"IMU Yaw Data: {robot.get_sensor_data('imu_yaw')}")
# print(f"IMU Pitch Data: {robot.get_sensor_data('imu_pitch')}")
# print(f"IMU Roll Data: {robot.get_sensor_data('imu_roll')}")
print(f"t1: {robot.get_sensor_data('time_1')}")
print(f"t2: {robot.get_sensor_data('time_2')}")
print(f"Simulated Robot Position: {x_robot_actual, y_robot_actual, z_robot_actual}")
# print(f"Calculated Possible Robot Position 1: {robot_position[0]}")
# print(f"Calculated Possible Robot Position 2: {robot_position[1]}")
print(f"Calculated Robot Position 1 in Global Frame: {robot_position_global}")
# print(f"Calculated Robot Position 2 in Global Frame: {robot_position2_global}")
print(f"Error: {error}")


