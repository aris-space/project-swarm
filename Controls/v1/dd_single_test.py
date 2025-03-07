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

import sys
import threading

import zmq


def zmq_interface(socket, llc):

    while True:
        #  Wait for next request from client
        message = socket.recv()
        print(f"Received request: {message}")

        #  Do some 'work'
        time.sleep(1)
        #llc.update_tar

        #  Send reply back to client
        socket.send_string("World")



def send_input():
    while True:
        key = sys.stdin.read(1)  # Read one character at a time
        if key == '1':
            ser.write(b'\n1')
            print("Sent: 1")
        elif key == '0':
            ser.write(b'\n0')
            print("Sent: 0")
        elif key == '9':
            running = not running
            print("Motors off: 9")

def receive_input(orientation, quat_w_offsets, quat, angular_rates, calibration):
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='replace').strip()
        if "Orientation" in line:
            values = line.split(":")[-1].strip()
            orientation = np.array([float(x) for x in values.split(",")])
            #print("Orientation:", orientation)
        elif "Quaternion rel. to offsets" in line:
            values = line.split(":")[-1].strip()
            quat_w_offsets = np.array([float(x) for x in values.split(",")])
            #print("Quaternion relative to offsets:", quat_w_offsets)
        elif "Quaternion:" in line and "rel. to offsets" not in line:
            values = line.split(":")[-1].strip()
            quat = np.array([float(x) for x in values.split(",")])
            #print("Quaternion:", quat)
        elif "Angular Rates (x,y,z):" in line:
            values = line.split(":")[-1].strip()
            #print(values)
            #print([str(x) for x in values.split(",")])
            angular_rates = np.array([float(x) for x in values.split(",") if x.strip() != ""])
            #print("Angular Rates (x,y,z):", angular_rates)
        elif "Calibration" in line:
            values = line.split(":")[-1].strip()
            calibration = np.array([int(x) for x in values.split(",")[-1].strip()])
            #print("Calibration:", calibration) 
        else:
            pass
    
    return orientation, quat_w_offsets, quat, angular_rates, calibration



if __name__ == "__main__":

    #empty log file
    open('total_state.log', 'w').close()

    #initialize llc with initial state and store time 
    llc = LLC2(CONSTANTS['pid_params'], CONSTANTS['init_params'])
    sca = SCA(llc)
    last_update = time.time()

    #3x2 array for roll, pitch, yaw and their rates from the init_params
    angular_rates = np.zeros(3)
    #initialize variables
    orientation = np.zeros(3)
    quat_w_offsets = np.zeros(4)
    quat = np.array([])
    calibration = np.array([])

    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    ser.flush()

    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:6000")



    with open('total_state.log', 'ab') as log:


        threading.Thread(target=send_input, daemon=True).start()
        threading.Thread(target=zmq_interface, args=(socket, llc), daemon=True).start()

        print("ready")

        running = True

        i=0

        
        while True:

            i+=1


            if running:

                #planner updates (1000 should be 10s due to 100Hz controller & IMU freq.)

                #set up serial connection

                #Start input thread


                for j in range(6):
                    orientation, quat_w_offsets, quat, angular_rates, calibration = receive_input(orientation, quat_w_offsets, quat, angular_rates, calibration)

                    #if True:#(angular_rates.size == 3 and quat_w_offsets.size == 4 and orientation.size == 3):
                    
                llc.update_actual_local_rates(angular_rates[0],angular_rates[1],angular_rates[2])
                    #pass controller the new angular rates



                llc.update_global_orientation_w_quat_sf(quat_w_offsets)

                if(i%100 == 0):
                    print(f"Quaternion {quat_w_offsets}")
                    print(f"Euler {orientation}")
                    print(f"Torques {torquex} {torquey} {torquez}")#print(torquex,torquey,torquez, quat_w_offsets, orientation) #zyx euler angles
                    print(f"Motor signals {sca.motor_signals}")
                    print(f"time {int(time.time()-last_update)}")
  

                torquex, torquey, torquez = llc.update_w_mode1(time.time() - last_update)
                    #calculate the torques


                last_update = time.time()

                    
                sca.update_motor_thrusts_manual(torquex, torquey, torquez)

                    #log rate rates in a log file
                np.savetxt(log, np.array([[orientation[0], sca.motor_signals[0]]]), delimiter=',')



            else:
    
                sca.update_motor_thrusts_manual(0, 0, 0)
                last_update = time.time()
