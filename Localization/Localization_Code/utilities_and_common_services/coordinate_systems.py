# this file contains initialization of all coordinate systems: global (fixed beacon) & local (individual agents)
# it also contains functions to convert points between global and local coordinate systems
# the global coordinate system is fixed at the origin (0,0,0)
# the local coordinate system is defined by the robot's position and orientation
# the orientation is represented by the roll, pitch, and yaw angles

import numpy as np 

class GlobalCoordinateSystem:
    def __init__(self, origin = (0,0,0)):
        """
        Initializes the global coordinate system with a given origin.
        
        :param origin: Tuple representing the global origin coordinates (x, y, z).
        """
        self.origin = origin # global origin: fixed point in space (beacon)
    
    def get_origin(self):
        # returns the origin of the global coordinate system
        return self.origin


class LocalCoordinateSystem:
    def __init__(self, robot_position, robot_orientation):
        """
        Initializes the local coordinate system for a robot.
        
        :param robot_position: Tuple representing the robot's position coordinates (x, y, z).
        :param robot_orientation: Tuple representing the robot's orientation (roll, pitch, yaw).
        """
        self.robot_position = robot_position
        self.orientation = robot_orientation

        # compute the rotation matrix from the orientation
        self.rotation_matrix = self.compute_rotation_matrix(robot_orientation)
    
    def compute_rotation_matrix(self, orientation):
        yaw, pitch, roll = orientation
         # Rotation matrix for yaw (z-axis rotation)
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # Rotation matrix for pitch (y-axis rotation)
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        # Rotation matrix for roll (x-axis rotation)
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        
        # Combined rotation matrix (yaw, pitch, roll)
        rotation_matrix = np.dot(Rz, np.dot(Ry, Rx))
        
        return rotation_matrix
    
    def convert_to_local(self, global_point):
        """
        Converts a point from global to local coordinates.
        
        :param global_point: Tuple representing the global point coordinates (x, y, z).
        :return: Tuple representing the local point coordinates (x, y, z).
        """
        x_global, y_global, z_global = global_point
        x_robot, y_robot, z_robot = self.robot_position 

        # subtract the robot's position from the global position to get the local position
        local_position = np.array([x_global - x_robot, y_global - y_robot, z_global - z_robot])

        # rotate the local position using the rotation matrix to convert from global to local 
        local_position_rotated = np.dot(np.linalg.inv(self.rotation_matrix), local_position)
        
        return tuple(local_position_rotated)
    
    def convert_to_global(self, local_point):
        """
        Converts a point from local to global coordinates.
        
        :param local_point: Tuple representing the local point coordinates (x, y, z).
        :return: Tuple representing the global point coordinates (x, y, z).
        """
        x_local, y_local, z_local = local_point
        x_robot, y_robot, z_robot = self.robot_position

        # rotate the local point using the rotation matrix to convert from local to global
        local_point_rotated = np.dot(self.rotation_matrix, np.array([x_local, y_local, z_local]))

        # add the robot's position to the rotated local point to get the global position
        global_position = np.array([x_robot, y_robot, z_robot]) + local_point_rotated
        
        return tuple(global_position)

def calc_robot_global_position(beacon_global_x, beacon_global_y, beacon_global_z, local_x, local_y, local_z, theta_robot):
    # Rotation matrix
    cos_theta = np.cos(theta_robot)
    sin_theta = np.sin(theta_robot)
    
    # Rotate and translate
    robot_global_x = beacon_global_x - (local_x * cos_theta - local_y * sin_theta)
    robot_global_y = beacon_global_y - (local_x * sin_theta + local_y * cos_theta) 
    robot_global_z = beacon_global_z - local_z
    
    return float(robot_global_x), float(robot_global_y), robot_global_z

def transform_to_global(local_x, local_y, local_z, robot_global_x, robot_global_y, robot_global_z, theta_robot):
    """
    Transforms coordinates from the robot's local frame to the global frame.

    Parameters:
    - local_x, local_y: Coordinates in the robot's local frame.
    - robot_global_x, robot_global_y: Robot's position in the global frame.
    - theta_robot: Robot's orientation (yaw angle) in radians.

    Returns:
    - global_x, global_y: Coordinates in the global frame.
    """
    cos_theta = np.cos(theta_robot)
    sin_theta = np.sin(theta_robot)

    global_x = robot_global_x + (local_x * cos_theta - local_y * sin_theta)
    global_y = robot_global_y + (local_x * sin_theta + local_y * cos_theta)
    global_z = robot_global_z + local_z

    return global_x, global_y, global_z

def transform_to_local(global_x, global_y, global_z, robot_global_x, robot_global_y, robot_global_z, theta_robot):
    """
    Transforms coordinates from the global frame to the robot's local frame.

    Parameters:
    - global_x, global_y: Coordinates in the global frame.
    - robot_global_x, robot_global_y: Robot's position in the global frame.
    - theta_robot: Robot's orientation (yaw angle) in radians.

    Returns:
    - local_x, local_y: Coordinates in the robot's local frame.
    """
    cos_theta = np.cos(theta_robot)
    sin_theta = np.sin(theta_robot)

    local_x = (global_x - robot_global_x) * cos_theta + (global_y - robot_global_y) * sin_theta
    local_y = -(global_x - robot_global_x) * sin_theta + (global_y - robot_global_y) * cos_theta
    local_z = global_z - robot_global_z

    return local_x, local_y, local_z
