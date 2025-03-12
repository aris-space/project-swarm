import serial
import numpy as np
import keyboard
import sys
import threading

j=0
k=0
l=0
m=0



def send_input():
    while True:
        key = sys.stdin.read(1)  # Read one character at a time
        if key == '1':
            ser.write(b'\n1')
            print("Sent: 1")
        elif key == '0':
            ser.write(b'\n0')
            print("Sent: 0")





def receive_input_xsens(ser, quat_offsets, quat, angular_rates, acceleration):

    global j,k,l,m

    if ser.in_waiting > 0:
        print(j, k, l, m)
        line = ser.readline().decode('utf-8', errors='replace').strip()
        if "QR" in line:
            j+=1
            quat_offsets = np.array(line[3:].split(','), dtype = float)
            if (j%50 == 0): print("Quaternion relative to offsets:", quat_offsets)
        elif 'Q' in line:
            k+=1
            quat = np.array(line[2:].split(','), dtype = float)
            if (k%50 == 0): print("Quaternion:", quat)
        elif "R" in line and "Q" not in line:
            l+=1
            angular_rates = np.array(line[2:].split(','), dtype = float)
            if (l%50 == 0): print("Angular Rates (x,y,z):", angular_rates)
        elif "A" in line:
            m+=1
            acceleration = np.array(line[2:].split(','), dtype = float)
            if (m%50 == 0): print("Acceleration", acceleration)
        else:
            print("Unknown line:", line)


if __name__ == "__main__":
    
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    ser.flush()

    quat_offsets = np.array([])
    quat = np.array([])
    angular_rates = np.array([])
    acceleration = np.array([])

    #Start input thread
    threading.Thread(target=send_input, daemon=True).start()

    while True:
        receive_input_xsens(ser, quat_offsets, quat, angular_rates, acceleration)


"""
#not used currently, this is for bno055


if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    ser.flush()

    orientation = np.array([])
    quat_offsets = np.array([])
    quat = np.array([])
    calibration = np.array([])
    i=0
    j=0
    k=0
    l=0
    m=0

    print("hi")


    #Start input thread
    threading.Thread(target=send_input, daemon=True).start()

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if "Orientation" in line:
                i+=1
                values = line.split(":")[-1].strip()
                orientation = np.array([float(x) for x in values.split(",")])
                if (i%50 == 0): print("Orientation:", orientation) 
            elif "Quaternion rel. to offsets" in line:
                j+=1
                values = line.split(":")[-1].strip()
                quat_offsets = np.array([float(x) for x in values.split(",")])
                if (j%50 == 0): print("Quaternion relative to offsets:", quat_offsets)
            elif "Quaternion:" in line and "rel. to offsets" not in line:
                k+=1
                values = line.split(":")[-1].strip()
                quat = np.array([float(x) for x in values.split(",")])
                if (k%50 == 0): print("Quaternion:", quat)
            elif "Calibration" in line:
                l+=1
                values = line.split(":")[-1].strip()
                calibration = np.array([int(x) for x in values.split(",")])
                if (l%50 == 0): print("Calibration:", calibration)
            elif "Angular Rates (x,y,z):" in line:
                m+=1
                values = line.split(":")[-1].strip()
                #print(values)
                #print([str(x) for x in values.split(",")])
                angular_rates = np.array([float(x) for x in values.split(",") if x.strip() != ""])
                if (m%50 == 0): print("Angular Rates (x,y,z):", angular_rates)
            else:
                print("Unknown line:", line)

        if keyboard.is_pressed('1'):
            ser.write(b'\n')  # Send newline
            ser.write(b'1')   # Send '1'
            print("Sent: 1")
        elif keyboard.is_pressed('0'):
            ser.write(b'\n')  # Send newline
            ser.write(b'0')   # Send '1'
            print("Sent: 0")

        



"""