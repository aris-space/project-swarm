import unittest
import numpy as np
from single_agent_controller.controllers.low_level_ctrl_2 import LLC2
from utils.constants2 import *
from utils.constants_loader import load_yaml

c = load_yaml('constants.yaml')

class TestLLC(unittest.TestCase):

    def setUp(self):

        llc_freq = 50
        self.controller = LLC2(c['pid_params'], c['init_params'])
        # Mock waypoints
        self.controller.global_orientation_target_quat = self.controller.euler_zyx_to_quaternion(0.0, 0.0, 0.0)

    def test_quaternion_conjugate(self):
        q = np.array([0.7071, 0.7071, 0.0, 0.0])
        q_conj = self.controller.quaternion_conjugate(q)
        expected = np.array([0.7071, -0.7071, -0.0, -0.0])
        np.testing.assert_array_almost_equal(q_conj, expected, decimal=4)

    def test_normalize_quaternion(self):
        q = np.array([2.0, 0.0, 0.0, 0.0])
        q_norm = self.controller.normalize_quaternion(q)
        expected = np.array([1.0, 0.0, 0.0, 0.0])
        np.testing.assert_array_almost_equal(q_norm, expected)


    def test_euler_zyx_to_quaternion(self):
        roll, pitch, yaw = 0.0, 0.0, np.pi / 2
        q = self.controller.euler_zyx_to_quaternion(roll, pitch, yaw)
        expected = np.array([0.7071, 0.0, 0.0, 0.7071])
        np.testing.assert_array_almost_equal(q, expected, decimal=4)

    

    def test_quaternion_multiply(self):
        q1 = np.array([0.7071, 0.0, 0.0, 0.7071])
        q2 = np.array([0.7071, 0.0, 0.0, -0.7071])
        result = self.controller.quaternion_multiply(q1, q2)
        expected = np.array([0.0, 0.0, 0.0, -1.0])
        np.testing.assert_array_almost_equal(result, expected, decimal=4)

    def test_quaternion_to_euler_zyx(self):
        q = np.array([0.7071, 0.0, 0.0, 0.7071])
        euler = self.controller.quaternion_to_euler_zyx(q)
        expected = np.array([0.0, 0.0, np.pi / 2])
        np.testing.assert_array_almost_equal(euler, expected, decimal=4)


    def test_from_rotvec_to_quaternion(self):
        rotvec = np.array([0.0, 0.0, np.pi / 2])
        q = self.controller.from_rotvec_to_quaternion(rotvec)
        expected = np.array([0.0, 0.0, 0.7071, 0.7071])
        np.testing.assert_array_almost_equal(q, expected, decimal=4)


    def test_calculate_error_quaternion(self):
        self.controller.global_orientation_estimate_quat = self.controller.euler_zyx_to_quaternion(0.0, 0.0, 0.0)
        self.controller.global_orientation_target_quat = self.controller.euler_zyx_to_quaternion(0.0, 0.0, np.pi / 2)
        q_error = self.controller.calculate_error_quaternion()
        expected = (-1)*self.controller.euler_zyx_to_quaternion(0.0, 0.0, np.pi / 2)
        np.testing.assert_array_almost_equal(q_error, expected, decimal=4)

    def test_calculate_local_angle_error(self):
        self.controller.global_orientation_estimate_quat = self.controller.euler_zyx_to_quaternion(0.0, 0.0, 0.0)
        self.controller.global_orientation_target_quat = self.controller.euler_zyx_to_quaternion(0.0, 0.0, np.pi / 2)
        local_error = self.controller.calculate_local_angle_error()
        expected = np.array([0.0, 0.0, -1.4142])
        np.testing.assert_array_almost_equal(local_error, expected, decimal=4)


    def test_update_angle_pids(self):
        self.controller.calculate_local_angle_error = lambda: np.array([0.1, -0.1, 0.2])
        self.controller.update_angle_pids()
        self.assertAlmostEqual(self.controller.desired_local_roll_rate, 0.1)
        self.assertAlmostEqual(self.controller.desired_local_pitch_rate, -0.1)
        self.assertAlmostEqual(self.controller.desired_local_yaw_rate, 0.2)



    def test_update_angle_rate_pids(self):
        self.controller.desired_local_roll_rate = 0.5
        self.controller.desired_local_pitch_rate = -0.8
        self.controller.desired_local_yaw_rate = 0.0
        self.controller.actual_local_roll_rate = 0.0
        self.controller.actual_local_pitch_rate = 0.0
        self.controller.actual_local_yaw_rate = 0.0
        torques = self.controller.update_angle_rate_pids()
        self.assertEqual(len(torques), 3)
        self.assertAlmostEqual(torques[0], 0.5)
        self.assertAlmostEqual(torques[1], -0.8)
        self.assertAlmostEqual(torques[2], 0.0)

         

    def test_update_actual_local_rates(self):
        self.controller.update_actual_local_rates(0.1, -0.1, 0.2)
        self.assertAlmostEqual(self.controller.actual_local_roll_rate, 0.1)
        self.assertAlmostEqual(self.controller.actual_local_pitch_rate, -0.1)
        self.assertAlmostEqual(self.controller.actual_local_yaw_rate, 0.2)



    def test_update_global_orientation_estimate_quat(self):
        self.controller.global_orientation_estimate_quat = self.controller.euler_zyx_to_quaternion(0.0, 0.0, 0.0)
        self.controller.actual_local_roll_rate = 0.0
        self.controller.actual_local_pitch_rate = 0.0
        self.controller.actual_local_yaw_rate = np.pi / 2
        self.controller.update_global_orientation_estimate_quat(dt=1.0)
        expected = self.controller.euler_zyx_to_quaternion(0.0, 0.0, np.pi / 2)
        np.testing.assert_array_almost_equal(self.controller.global_orientation_estimate_quat, expected, decimal=4)


          
if __name__ == '__main__':
    unittest.main()