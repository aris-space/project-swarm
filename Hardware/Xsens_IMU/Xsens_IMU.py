import sys
import numpy as np
import serial
import keyboard

class Xsens_IMU:

    def __init__(self):
        #self.swarm_storage = swarm_storage

        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.ser.flush()

        self.quat_offsets = np.array([])
        self.quat = np.array([])
        self.angular_rates = np.array([])
        self.acceleration = np.array([])

    def send_input(self):
        key = sys.stdin.read(1)  # Read one character at a time
        if key == '1':
            self.ser.write(b'\n1')
            print("Sent: 1")
        elif key == '0':
            self.ser.write(b'\n0')
            print("Sent: 0")

    def receive_input_xsens(self):
        for i in range(170):
            if self.ser.in_waiting > 0:
                i+=1
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
                if "QR" in line:
                    self.quat_offsets = np.array(line[3:].split(','), dtype=float)
                    # Instead of returning, the value is stored.
                elif 'Q' in line:
                    self.quat = np.array(line[2:].split(','), dtype=float)
                elif "R" in line and "Q" not in line:
                    self.angular_rates = np.array(line[2:].split(','), dtype=float)
                elif "A" in line:
                    self.acceleration = np.array(line[2:].split(','), dtype=float)
                else:
                    print("Unknown line:", line)

    # --- Getter methods for sensor values ---
    def get_quat_offsets(self):
        return self.quat_offsets

    def get_quaternion(self):
        return self.quat

    def get_angular_rates(self):
        return self.angular_rates

    def get_acceleration(self):
        return self.acceleration
