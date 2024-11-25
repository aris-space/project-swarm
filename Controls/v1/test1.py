import unittest
import numpy as np
from single_agent_controller.controllers.low_level_ctrl import LLC
from utils.constants2 import *
from utils.constants_loader import load_yaml

c = load_yaml('constants.yaml')

class TestLLC(unittest.TestCase):

    def setUp(self):

        llc_freq = 50
        self.llc = LLC(c['pid_params'], c['init_params'], llc_freq)

    def test_update_orientation_no_rotation(self):
        global_quat = np.array([0.0, 0.0, 0.0, 1.0])
        roll_rate = 0.0
        pitch_rate = 0.0
        yaw_rate = 0.0
        dt = 0.1
        new_global_quat, roll, pitch, yaw = self.llc.update_orientation(global_quat, roll_rate, pitch_rate, yaw_rate, dt)
        expected_quat = global_quat
        np.testing.assert_array_almost_equal(new_global_quat, expected_quat)
        self.assertAlmostEqual(roll, 0.0)
        self.assertAlmostEqual(pitch, 0.0)
        self.assertAlmostEqual(yaw, 0.0)

    def test_update_orientation_with_rotation(self):
        global_quat = np.array([0.0, 0.0, 0.0, 1.0])
        roll_rate = np.pi / 2  # 90 degrees per second
        pitch_rate = 0.0
        yaw_rate = 0.0
        dt = 1.0  # 1 second
        new_global_quat, roll, pitch, yaw = self.llc.update_orientation(global_quat, roll_rate, pitch_rate, yaw_rate, dt)
        expected_roll = np.pi / 2
        self.assertAlmostEqual(roll, expected_roll)
        self.assertAlmostEqual(pitch, 0.0)
        self.assertAlmostEqual(yaw, 0.0)

    # complex quaternion update with multiple rotations and inital global rotation not 0 for roll, pitch, yaw
    def test_update_orientation_with_rotation_complex(self):
        global_quat = np.array([0.0, 0.0, 0.0, 1.0])
        roll_rate = np.pi / 2
        pitch_rate = np.pi / 4
        yaw_rate = np.pi / 8

        #calculate complex quaternion update with roll_rate, pitch_rate, yaw_rate and global_quat ourselves
        # and compare with the output of the function


        dt = 1.0
        new_global_quat, roll, pitch, yaw = self.llc.update_orientation(global_quat, roll_rate, pitch_rate, yaw_rate, dt)
        self.llc.orientation_estimate_quat == 


        expected_roll = np.pi / 2
        expected_pitch = np.pi / 4
        expected_yaw = np.pi / 8
        self.assertAlmostEqual(roll, expected_roll)
        self.assertAlmostEqual(pitch, expected_pitch)
        self.assertAlmostEqual(yaw, expected_yaw)


    def test_update_from_IMU_np_arr_valid_input(self):
        angle_state = np.array([[0.0, np.pi / 4],
                                [0.0, np.pi / 4],
                                [0.0, np.pi / 4]])
        depth_state = np.array([10.0, 1.0])
        dt = 0.1
        self.llc.update_from_IMU_np_arr(angle_state, depth_state, dt)
        # Check if orientation quaternion is updated
        self.assertTrue(self.llc.is_valid_quaternion(self.llc.orientation_estimate_quat))
        # Since internal variables are not accessible, we assume the method runs without errors

    def test_update_from_IMU_np_arr_invalid_angle_state(self):
        angle_state = np.array([[0.0],
                                [0.0],
                                [0.0]])  # Invalid shape
        depth_state = np.array([10.0, 1.0])
        dt = 0.1
        with self.assertRaises(ValueError):
            self.llc.update_from_IMU_np_arr(angle_state, depth_state, dt)

    def test_update_from_IMU_np_arr_invalid_depth_state(self):
        angle_state = np.array([[0.0, 0.0],
                                [0.0, 0.0],
                                [0.0, 0.0]])
        depth_state = np.array([10.0])  # Invalid shape
        dt = 0.1
        with self.assertRaises(ValueError):
            self.llc.update_from_IMU_np_arr(angle_state, depth_state, dt)

if __name__ == '__main__':
    unittest.main()