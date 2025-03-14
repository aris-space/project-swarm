# drone_instance.py
import threading
import time
import random
import json
import numpy as np
import zmq
import csv

# --- Imports from your existing modules ---
from MissionPlanner.swarm_storage import SwarmSensorDataSingleton, DroneRawData, DroneFilteredData
from Communication.transceive_functions import LoRa_Transceiver
from Localization_New.Waterlinked.waterlinked import WaterLinked
from Hardware.Integrated_Sensors.Drone_Sensors import TemperaturePressure
from Controls.v1.single_agent_controller.controllers.low_level_ctrl_2 import LLC2
from Controls.v1.utils import CONSTANTS
from Controls.v1.utils.constants2 import MAX_VEL

# --- Import DataLogger from MainHelper.utils_functions ---
from MainHelper.utils_functions import DataLogger

class DroneInstance:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, drone_id, *args, **kwargs):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super(DroneInstance, cls).__new__(cls)
        return cls._instance

    def __init__(self, drone_id):
        # Ensure __init__ only runs once.
        if hasattr(self, '_initialized'):
            return
        self._initialized = True

        # --- Basic Attributes ---
        self.drone_id = drone_id

        # --- Data Storage ---
        self.swarm_storage = SwarmSensorDataSingleton()  # Holds raw and filtered drone data

        # --- Data Logger ---
        # The DataLogger is imported from MainHelper.utils_functions.
        # It will create/overwrite "data_log.csv" and write a header.
        self.data_logger = DataLogger("data_log.csv")

        # --- Simple Logger ---
        self.log_file = "drone_instance.log"
        with open(self.log_file, "w") as f:
            f.write("DroneInstance Logger initialized\n")
        self.logger = self.simple_logger

        # --- Sensors & Localization ---
        self.localization_sensor = WaterLinked(base_url="http://192.168.8.108", poll_interval=1.0, test_mode=True)
        self.temperature_sensor = TemperaturePressure()  # e.g. temperature/pressure sensor

        # --- Communications ---
        self.lora_transceiver = LoRa_Transceiver(device_id=drone_id, slot_duration=4)

        # --- Control ---
        self.controller = LLC2(CONSTANTS['pid_params'], CONSTANTS['init_params'], MAX_VEL)

        self.logger(f"DroneInstance initialized with drone_id: {drone_id}")

    def simple_logger(self, message):
        timestamp = time.time()
        entry = f"{timestamp}: {message}"
        with open(self.log_file, "a") as f:
            f.write(entry + "\n")
        print(entry)

    # ------------- High-Level Thread Methods -----------------
    def sensor_thread_method(self):
        """
        This method is the target for the sensor thread.
        It reads localization (and optionally other sensor) data and updates the swarm storage.
        """
        sensor_poll_interval = 0.1  # Polling frequency in seconds
        while True:
            try:
                # Get localization data (this may block up to 2 seconds)
                localization = self.localization_sensor.get_latest_position(timeout=2)
                if localization is None:
                    self.logger("Waterlinked: No new data")
                else:
                    self.swarm_storage.raw_drones[self.drone_id].update_position(values=localization)
                    self.logger(f"Updated waterlinked: {localization}")
            except Exception as e:
                self.logger(f"Error in sensor_thread_method: {e}")

            # Uncomment and extend for temperature/depth sensor processing:
            """
            try:
                temperature = self.temperature_sensor.get_temperature()  # Or similar method
                if temperature is None:
                    self.logger("Temperature sensor: No new data")
                else:
                    self.swarm_storage.raw_drones[self.drone_id].update_temperature(temperature)
                    self.logger(f"Updated temperature: {temperature}")
            except Exception as e:
                self.logger(f"Error reading temperature sensor: {e}")
            """
            time.sleep(sensor_poll_interval)

    def communication_thread_method(self):
        """
        This method is the target for the communication thread.
        It listens for incoming LoRa messages and, at a random moment within each slot, transmits current drone data.
        """
        slot_duration = 0.2  # Slot duration in seconds
        while True:
            slot_start = time.time()
            transmit_time = slot_start + random.uniform(0, slot_duration)
            transmitted = False
            while time.time() - slot_start < slot_duration:
                try:
                    # Listen for incoming messages.
                    msg = self.lora_transceiver.receive_LoRa()
                    if msg is not None:
                        self.logger(f"Received message: {msg}")
                        # Update the swarm storage with the received data.
                        # (Dummy example; update as needed.)
                        self.swarm_storage.update_from_received(msg)
                except Exception as e:
                    self.logger(f"Communication error while receiving: {e}")

                # At a random moment, transmit current data.
                if not transmitted and time.time() >= transmit_time:
                    try:
                        localization = [52.1212, 15.232, 89.1223]
                        self.swarm_storage.filtered_drones[self.drone_id].update_position(values=localization)
                        message_to_send = self.swarm_storage.filtered_drones[self.drone_id].format_drone_data()
                        self.lora_transceiver.transmit_LoRa(message_to_send)
                        transmitted = True
                    except Exception as e:
                        self.logger(f"Communication error during transmission: {e}")
                time.sleep(0.01)

    def control_thread_method(self):
        """
        This method is the target for the control thread.
        It runs the control loop, reads the current drone state, and updates the controller.
        """
        control_poll_interval = 0.1  # Control loop frequency

        # ZeroMQ setup for publishing control data (if needed)
        context = zmq.Context()
        socket = context.socket(zmq.PUB)
        socket.bind("tcp://*:5555")
        time_start = time.time()

        while True:
            try:
                self.logger("Initializing controller in control_thread")
                # Optionally clear previous log file.
                open('data_log.log', 'w').close()
                # For demonstration, using a dummy waypoint.
                waypoints = [{"x": 10, "y": 20, "z": 30, "yaw": 0, "pitch": 0, "roll": 0}]
                self.controller.global_position_target = np.array([waypoints[0]['x'], waypoints[0]['y'], waypoints[0]['z']])
                self.controller.global_orientation_target_quat = self.controller.euler_zyx_to_quaternion(
                    waypoints[0]['yaw'], waypoints[0]['pitch'], waypoints[0]['roll']
                )
                num_steps = 1000

                # Instead of using np.savetxt, you could also log sensor data via the DataLogger.
                # For example, if you had a sensor_data instance with a to_list() method:
                # self.data_logger.log_data(sensor_data)
                with open('data_log.log', 'ab') as log:
                    for i in range(num_steps):
                        self.logger("Control loop iteration")
                        pos_x = self.swarm_storage.raw_drones[self.drone_id].position.x
                        pos_y = self.swarm_storage.raw_drones[self.drone_id].position.y
                        pos_z = self.swarm_storage.raw_drones[self.drone_id].position.z

                        desired_x_vel, desired_y_vel, desired_z_vel, finished = self.controller.update_w_mode6()

                        # Here, we log control data using numpy, but you could also call:
                        # self.data_logger.log_data(sensor_data) if you prepare a sensor_data object.
                        np.savetxt(log, np.array([desired_x_vel, desired_y_vel, desired_z_vel, pos_x, pos_y, pos_z]).reshape(1, -1), delimiter=',')

                        if finished:
                            self.logger("All waypoints finished")
                            break

                        self.controller.update_global_position(pos_x, pos_y, pos_z)
                        time.sleep(0.1)

                        elapsed_time = time.time() - time_start
                        value = {
                            "timestamp": elapsed_time,
                            "variables": {
                                "pos_x": pos_x,
                                "pos_y": pos_y,
                                "pos_z": pos_z,
                                "des_vx": desired_x_vel,
                                "des_vy": desired_y_vel,
                                "des_vz": desired_z_vel,
                            },
                        }
                        socket.send_string("controls", zmq.SNDMORE)
                        socket.send_string(json.dumps(value))
                data = np.loadtxt('data_log.log', delimiter=',')
                socket.close()
                context.term()
            except Exception as e:
                self.logger(f"Control error: {e}")

            time.sleep(control_poll_interval)
