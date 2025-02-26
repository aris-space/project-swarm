import serial
import numpy as np
import keyboard

if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.flush()

    orientation = np.array([])
    quat_offsets = np.array([])
    quat = np.array([])
    calibration = np.array([])

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if "Orientation" in line:
                values = line.split(":")[-1].strip()
                orientation = np.array([float(x) for x in values.split(",")])
                print("Orientation:", orientation)
            elif "Quaternion rel. to offsets" in line:
                values = line.split(":")[-1].strip()
                quat_offsets = np.array([float(x) for x in values.split(",")])
                print("Quaternion relative to offsets:", quat_offsets)
            elif "Quaternion:" in line and "rel. to offsets" not in line:
                values = line.split(":")[-1].strip()
                quat = np.array([float(x) for x in values.split(",")])
                print("Quaternion:", quat)
            elif "Calibration" in line:
                values = line.split(":")[-1].strip()
                calibration = np.array([int(x) for x in values.split(",")])
                print("Calibration:", calibration)
            elif "Angular Rates (x,y,z):" in line:
                values = line.split(":")[-1].strip()
                angular_rates = np.array([int(x) for x in values.split(",")])
                print("Angular Rates (x,y,z):", angular_rates)
            else:
                print("Unknown line:", line)
                #print(line)

        if keyboard.is_pressed('1'):
            ser.write(b'\n')  # Send newline
            ser.write(b'1')   # Send '1'
            print("Sent: 1")
        elif keyboard.is_pressed('0'):
            ser.write(b'\n')  # Send newline
            ser.write(b'0')   # Send '1'
            print("Sent: 0")




