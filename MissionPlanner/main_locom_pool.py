# Imports
import sys
import time
import threading
import random
import numpy as np
import serial
import keyboard
import zmq
import json

sys.path.insert(0, "/shared_folder/project-swarm")

from Communication.transceive_functions import *
from Localization_New.Waterlinked.waterlinked import *
from Utils.utils_functions import *
from MissionPlanner.swarm_storage import *
from Hardware.Integrated_Sensors.Drone_Sensors import *
from Hardware.Xsens_IMU.Xsens_IMU import *
from Controls.v1.single_agent_controller.controllers.low_level_ctrl_2 import *
from Controls.v1.utils import CONSTANTS
from Controls.v1.utils.constants2 import MAX_VEL

#from data_classes import *


# Global variables 
last_no_temperature_time = None
last_no_depth_time = None

#------THREADING FUNCTIONS------
def sensor_thread():
    """
    Continuously reads sensor data and updates swarm storage.
    Processes temperature, depth (from Keller_Sensor), and waterlinked localization data.
    - Temperature: Uses Keller_Sensor.get_temperature()
    - Sensor depth: Uses Keller_Sensor.get_depth() and stores it separately (via update_sensor_depth)
    - Waterlinked localization: Uses get_localization.get_latest_position(timeout=2)
      and stores the full position via update_position.
      
    If a sensor returns no data, a timestamp is recorded to indicate when the data stopped updating.
    """
    global last_no_temperature_time, last_no_depth_time
    sensor_poll_interval = 0.1  # Polling frequency in seconds

    while True:
        """
        # Process waterlinked localization data
        try:
            # This call may block for up to 2 seconds if no new localization data is available.
            localization = get_localization.get_latest_position(timeout=2)
            if localization is None:
                print("Waterlinked: No new data")
            else:
                swarm_data.raw_drones[drone_id].update_position(values=localization)
                print(f"Updated waterlinked: {localization}")
        except Exception as e:
            print(f"Error reading waterlinked localization: {e}")

        # Process temperature sensor data (from Keller_Sensor)
        try:
            temperature = Keller_Sensor.get_temperature()  # Expected to return a temperature value or None
            if temperature is None:
                if last_no_temperature_time is None:
                    last_no_temperature_time = time.time()
                    print("Temperature sensor: No new data received. Starting timer.")
            else:
                swarm_data.raw_drones[drone_id].update_temperature(temperature)
                last_no_temperature_time = None
                print(f"Updated temperature: {temperature}")
        except Exception as e:
            print(f"Error reading temperature sensor: {e}")

        # Process depth sensor data (from Keller_Sensor)
        try:
            depth = Keller_Sensor.get_depth()  # Expected to return a depth value or None
            if depth is None:
                if last_no_depth_time is None:
                    last_no_depth_time = time.time()
                    print("Depth sensor: No new data received. Starting timer.")
            else:
                # Store sensor depth separately; this method should be implemented in swarm_storage.
                #swarm_data.raw_drones[drone_id].update_sensor_depth(depth)
                swarm_data.raw_drones[drone_id].update_conductivity(depth)
                last_no_depth_time = None
                print(f"Updated sensor depth: {depth}")
        except Exception as e:
            print(f"Error reading depth sensor: {e}")

        time.sleep(sensor_poll_interval)
        """

def communication_thread():
    """
    Continuously handles inter-drone communication.
    In each 0.5 second slot, the drone:
    - Listens continuously for incoming messages,
    - Updates the swarm storage when new data is received,
    - And at a random moment within the slot, transmits its own formatted data.
    """
    slot_duration = 0.2 # Slot duration in seconds

    while True:
        slot_start = time.time()
        # Choose a random transmission time within the current slot.
        transmit_time = slot_start + random.uniform(0, slot_duration)
        transmitted = False
        while time.time() - slot_start < slot_duration:
            try:
                # Listen for incoming messages.
                msg = lora_transceiver.receive_LoRa()

                if msg is not None:
                    print(f"Received message: {msg}")
                    # For now, update the swarm data with dummy received data !!
                    #received_data = [
                    #    2, 1.23, 6.56, 7.89, 10.0, 5.0, 15.0, 22.5, 1.02, 543,
                    #    [2, 1, 1, 1, 1, 1, 1, 1, 0, 2]
                    #]
                    swarm_data.update_from_received(msg)
                
            except Exception as e:
                print(f"Communication error while receiving: {e}")

            # At the random moment in the slot, transmit the current drone data.
            if not transmitted and time.time() >= transmit_time:
                try:
                    # Extract the message to send from the swarm data.
                    localization = [52.1212, 15.232, 89.1223]
                    swarm_data.filtered_drones[drone_id].update_position(values=localization)
                    message_to_send = swarm_data.filtered_drones[drone_id].format_drone_data()
                    #message_to_send = swarm_data.filtered_drones[drone_id].format_drone_data()
                    lora_transceiver.transmit_LoRa(message_to_send)
                    transmitted = True
                except Exception as e:
                    print(f"Communication error during transmission: {e}")

            # Short sleep to avoid busy-waiting.
            time.sleep(0.01)

def control_thread():
    """
    Continuously runs the control loop.
    Reads current drone state from swarm storage and, in a full implementation,
    would run filters/controls to determine actuator commands.
    """
    control_poll_interval = 0.1  # Control loop frequency


    # ZeroMQ setup
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5555")  # Bind to a port PlotJuggler will connect to

    time_start = time.time()

    while True:
        try:     

            # Initialisiere den Low-Level Controller (LLC) mit den Konstanten aus CONSTANTS
            print("Initializing")
            open('data_log.log', 'w').close()
            
            llc = LLC2(CONSTANTS['pid_params'], CONSTANTS['init_params'], MAX_VEL)
            # Setze den einzelnen Waypoint als globales Ziel
            llc.global_position_target = np.array([waypoints[0]['x'], waypoints[0]['y'], waypoints[0]['z']])
            llc.global_orientation_target_quat = llc.euler_zyx_to_quaternion(
                waypoints[0]['yaw'], waypoints[0]['pitch'], waypoints[0]['roll']
            )
            
            num_steps = 1000
            
            with open('data_log.log', 'ab') as log:
                for i in range(0, num_steps):

                    print("hey")

                    pos_x = swarm_data.raw_drones[2].position.x
                    pos_y = swarm_data.raw_drones[2].position.y
                    pos_z = swarm_data.raw_drones[2].position.z

                    desired_x_vel, desired_y_vel, desired_z_vel, finished = llc.update_w_mode6()
                    
                    # Logge die Desired Velocities in das Array und in die separate Logdatei

                    np.savetxt(log, np.array([desired_x_vel, desired_y_vel, desired_z_vel, pos_x, pos_y, pos_z]).reshape(1, -1), delimiter=',')
                    
                    if finished:
                        print("alle waypoints finished")
                        break
                    
                    llc.update_global_position(pos_x, pos_y, pos_z)
                    time.sleep(0.1) 

                    elapsed_time = time.time() - time_start

                    value = {
                        "timestamp": elapsed_time,  # Zeit in Sekunden seit Scriptstart
                        "variables": {
                            "pos_x": pos_x,  
                            "pos_y": pos_y,  
                            "pos_z": pos_z,
                            },
                        }

                    # Sende Daten als JSON mit dem Thema "sincos"
                    socket.send_string("controls", zmq.SNDMORE)
                    socket.send_string(json.dumps(value))
                    
            data = np.loadtxt('data_log.log',delimiter=',')


            socket.close()
            context.term()

            """
            print(pos_x)
            #new_position = control_test.update_global_position(pos.x, pos.y, pos.z) #change format
            desired_log_filename = 'desired_velocities.log'
            open(desired_log_filename, 'w').close()

            
            start_time = time.time()  # Start the timer
            imu.receive_input_xsens()  # Process one line from the IMU
            end_time = time.time()  # Stop the timer
            execution_time = end_time - start_time 
            print(execution_time)

            accel_values = imu.get_acceleration()
            print(accel_values)
            angular_values = imu.get_angular_rates()
            quat_values = imu.get_quaternion()
            quat_offsets_values = imu.get_quat_offsets()

            swarm_data.filtered_drones[drone_id].update_acceleration(accel_values)
            print("Drone 1 acceleration:", swarm_data.filtered_drones[drone_id].acceleration)

            print("Drone 1 acceleration:", swarm_data.filtered_drones[drone_id].acceleration.ax)

            swarm_data.filtered_drones[drone_id].update_angular_rates(angular_values)
            print("Drone 1 angular rates:", swarm_data.filtered_drones[drone_id].angular_rates.roll_rate)

            swarm_data.filtered_drones[1].update_quaternion(quat_values)
            print("Drone 1 quaternion:", swarm_data.filtered_drones[1].quaternion.qy)

            swarm_data.filtered_drones[drone_id].update_quat_offsets(quat_offsets_values)
            print("Drone 1 quaternion offsets:", swarm_data.filtered_drones[drone_id].quat_offsets.qw)
            """

        except Exception as e:
            print(f"Control error: {e}")

        time.sleep(control_poll_interval)



"""Run code with: python aa_mission_planner.py --Drone_ID=<Drone_ID> in Terminal!!"""

if __name__ == "__main__":
    

    drone_id = get_drone_id()
    print(f"Drone ID: {drone_id}")

    # INITIALIZING the necessary objects

    raw_data = DroneRawData()
    filtered_data = DroneFilteredData()
    swarm_data = SwarmSensorData()  # Stores data for all drones

    lora_transceiver = LoRa_Transceiver(device_id=drone_id, slot_duration=4)
    get_localization = WaterLinked(base_url="http://192.168.8.108", poll_interval=1.0, test_mode=True)
    Keller_Sensor = TemperaturePressure() #need sudo pigpiod in terminal!!!!

    #GET LOCALIZATION: 

    #position_data = get_localization.get_latest_position(timeout = 2)
    #print(position_data)
    # New localization data: only the current position [x, y, z]
    position_data = [71.21, 62.10, 76.21]

    sensor_thread_obj = threading.Thread(target=sensor_thread, name="SensorThread", daemon=True)
    communication_thread_obj = threading.Thread(target=communication_thread, name="CommThread", daemon=True)

    pos_x = swarm_data.raw_drones[drone_id].position.x
    pos_y = swarm_data.raw_drones[drone_id].position.y
    pos_z = swarm_data.raw_drones[drone_id].position.z


    control_thread_obj = threading.Thread(target=control_thread, name="ControlThread", daemon=True)

    #sensor_thread_obj.start()
    communication_thread_obj.start()
    control_thread_obj.start()


    """
    drone_2_temperature = swarm_data.raw_drones[2].temperature
    drone_2_temp_fin = swarm_data.filtered_drones[2].temperature
    drone_2_temp_full = swarm_data.filtered_drones[2].format_drone_data()
    print(drone_2_temp_fin)
    print(drone_2_temperature)
    print(drone_2_temp_full)
    #control_thread_obj.start()
    """


    while True:
        time.sleep(1)





