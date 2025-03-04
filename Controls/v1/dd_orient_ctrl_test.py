from single_agent_controller.controllers.low_level_ctrl_2 import LLC2
from single_agent_controller.controllers.ca_system import SCA

from utils.waypoints import *
from utils import CONSTANTS
from utils.constants2 import *

import numpy as np

import serial
import numpy as np
import keyboard
import time


if __name__ == "__main__":

    #empty log file
    open('total_state.log', 'w').close()

    #initialize llc with initial state and store time 
    llc = LLC2(CONSTANTS['pid_params'], CONSTANTS['init_params'])
    last_update = time.time()

    #3x2 array for roll, pitch, yaw and their rates from the init_params
    angle_rates = np.zeros(3)

    with open('total_state.log', 'ab') as log:

        for i in range(1,1000):
            #planner updates (1000 should be 10s due to 100Hz controller & IMU freq.)

            #set up serial connection
            ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            ser.flush()

            #initialize variables
            orientation = np.array([])
            quat_w_offsets = np.array([])
            quat = np.array([])
            calibration = np.array([])

            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                if "Orientation" in line:
                    values = line.split(":")[-1].strip()
                    orientation = np.array([float(x) for x in values.split(",")])
                    print("Orientation:", orientation)
                elif "Quaternion rel. to offsets" in line:
                    values = line.split(":")[-1].strip()
                    quat_w_offsets = np.array([float(x) for x in values.split(",")])
                    print("Quaternion relative to offsets:", quat_w_offsets)
                elif "Quaternion:" in line and "rel. to offsets" not in line:
                    values = line.split(":")[-1].strip()
                    quat = np.array([float(x) for x in values.split(",")])
                    print("Quaternion:", quat)
                elif "Angular rates" in line:
                    values = line.split(":")[-1].strip()
                    angle_rates = np.array([float(x) for x in values.split(",")])
                    print("Angular rates:", angle_rates)
                elif "Calibration" in line:
                    values = line.split(":")[-1].strip()
                    calibration = np.array([int(x) for x in values.split(",")])
                    print("Calibration:", calibration) 
                else:
                    print(line)

                if angle_rates.size == 3 and quat.size == 4:
                    llc.update_actual_local_rates(angle_rates)
                    #pass controller the new angular rates

                    llc.global_orientation_estimate_quat = np.array(quat_w_offsets[1],quat_w_offsets[2],quat_w_offsets[3],quat_w_offsets[0])
                    #TODO: update the LLC with a function that reads scalar first quat and stores them scalar last and normalize the quaternion
                    #done

                    torquex, torquey, torquez = llc.update_w_mode1(time.time() - last_update)
                    #calculate the torques
                    #TODO: change update_w_mode1 to take real dt's (time.time() - last_update)
                    #done
                    
                    last_update = time.time()

                    #wait for 0.01 - (time.time() - last_update) seconds
                    time.sleep(0.01 - (time.time() - last_update))

                    SCA.update_motor_thrusts_manual(torquex,torquey,torquez)
                    #update the motor thrusts


            #log rate rates in a log file
            np.savetxt(log, angle_rates, delimiter=',')


            #listen for offsets turned on/off
            if keyboard.is_pressed('1'):
                ser.write(b'\n')  # Send newline
                ser.write(b'1')   # Send '1'
                print("Sent: 1")
            elif keyboard.is_pressed('0'):
                ser.write(b'\n')  # Send newline
                ser.write(b'0')   # Send '1'
                print("Sent: 0")