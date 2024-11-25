import unittest
import time

from single_agent_controller.controllers.pid_ctrl import PID

"""

class TestPID(unittest.TestCase):
    def test_pid_initialization(self):
        pid = PID(1.0, 0.1, 0.01)
        self.assertEqual(pid.kp, 1.0)
        self.assertEqual(pid.ki, 0.1)
        self.assertEqual(pid.kd, 0.01)

    def test_pid_update(self):
        pid = PID(1.0, 0.1, 0.01)
        output = pid.update(10, 0, dt=1)
        self.assertIsNotNone(output)

    def test_windup_max(self):
        pid = PID(1.0, 0.1, 0.01, windup_max=5)
        pid.previous_integral = 10
        pid.update(0, 0, dt=1)
        self.assertLessEqual(pid.previous_integral, 5)

    def test_saturation_max(self):
        pid = PID(10.0, 0.0, 0.0, saturation_max=5)
        output = pid.update(1, 0, dt=1)
        self.assertLessEqual(output, 5)

    def test_set_point_weighting(self):
        pid = PID(1.0, 0.1, 0.01, set_point_weighting=True, weight_b=0.5, weight_c=0.5)
        output = pid.update(10, 5, dt=1)
        self.assertIsNotNone(output)

    def test_negative_feedback(self):
        pid = PID(1.0, 0.1, 0.01)
        output = pid.update(-10, 0, dt=1)
        self.assertLess(output, 0)

    def test_zero_gain(self):
        pid = PID(0.0, 0.0, 0.0)
        output = pid.update(10, 0, dt=1)
        self.assertEqual(output, 0)

    def test_derivative_kick(self):
        pid = PID(1.0, 0.1, 0.01)
        output1 = pid.update(10, 0, dt=1)
        output2 = pid.update(10, 5, dt=1)
        self.assertNotEqual(output1, output2)

    def test_integral_windup(self):
        pid = PID(1.0, 0.1, 0.01, windup_max=5)
        for _ in range(10):
            pid.update(10, 0, dt=1)
        self.assertLessEqual(pid.previous_integral, 5)

    def test_combined_saturation_windup_set_point_weighting(self):
        pid = PID(1.0, 0.1, 0.01, saturation_max=5, windup_max=5, set_point_weighting=True, weight_b=0.5, weight_c=0.5)
        for _ in range(10):
            output = pid.update(10, 0, dt=1)
        self.assertLessEqual(output, 5)
        self.assertLessEqual(pid.previous_integral, 5)
        self.assertIsNotNone(output)

    def test_integrated_timing(self):
        pid = PID(1.0, 0.1, 0.01)
        pid.previous_time = time.time() - 1  # Simulate last update was 1 second ago
        pid.update(10, 0)
        time_1 = pid.previous_time
        dt1 = pid.previous_time - (pid.previous_time - 1)
        time.sleep(0.5)  # Wait for 0.5 seconds
        pid.update(10, 0)
        time_2 = pid.previous_time
        dt2 = pid.previous_time - time_1
        self.assertNotEqual(dt1, dt2)
        self.assertGreater(dt1, dt2)

    

if __name__ == '__main__':
    unittest.main()

"""