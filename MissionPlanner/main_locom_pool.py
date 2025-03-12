# Imports
import sys
import time
import threading
import random
sys.path.insert(0, "/shared_folder/project-swarm")

from Communication.transceive_functions import *
from Localization_New.Waterlinked.waterlinked import *
from Utils.utils_functions import *
from MissionPlanner.swarm_storage import *
from Hardware.Integrated_Sensors.Drone_Sensors import *
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


def communication_thread():
    """
    Continuously handles inter-drone communication.
    In each 0.5 second slot, the drone:
    - Listens continuously for incoming messages,
    - Updates the swarm storage when new data is received,
    - And at a random moment within the slot, transmits its own formatted data.
    """
    slot_duration = 0.5  # Slot duration in seconds

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
                    received_data = [
                        2, 1.23, 6.56, 7.89, 10.0, 5.0, 15.0, 22.5, 1.02, 543,
                        [2, 1, 1, 1, 1, 1, 1, 1, 0, 2]
                    ]
                    swarm_data.update_from_received(received_data)
            except Exception as e:
                print(f"Communication error while receiving: {e}")

            # At the random moment in the slot, transmit the current drone data.
            if not transmitted and time.time() >= transmit_time:
                try:
                    # Extract the message to send from the swarm data.
                    message_to_send = swarm_data.raw_drones[drone_id].format_drone_data()
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

    while True:
        try:
            accel_values = np.array([0.0092, 0.0072, -0.871])
            swarm_data.filtered_drones[drone_id].update_acceleration(accel_values)
            print("Drone 1 acceleration:", swarm_data.filtered_drones[drone_id].acceleration)

            angular_values = np.array([0.001, 0.002, 0.003])
            swarm_data.filtered_drones[drone_id].update_angular_rates(angular_values)
            print("Drone 1 angular rates:", swarm_data.filtered_drones[drone_id].angular_rates)

            quat_values = np.array([0.5, 0.5, 0.5, 0.5])
            swarm_data.filtered_drones[1].update_quaternion(quat_values)
            print("Drone 1 quaternion:", swarm_data.filtered_drones[1].quaternion)
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

    log_data_csv = DataLogger(f"drone_{drone_id}_log.csv") 

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
    control_thread_obj = threading.Thread(target=control_thread, name="ControlThread", daemon=True)

    #sensor_thread_obj.start()
    #communication_thread_obj.start()
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




