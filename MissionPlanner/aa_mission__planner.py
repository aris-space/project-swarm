# Imports
import sys
sys.path.insert(0, "/shared_folder/project-swarm")

from Communication.transceive_functions import *
from Localization_New.Waterlinked.waterlinked import *
from Utils.utils_functions import *
from MissionPlanner.swarm_storage import *
from Hardware.Integrated_Sensors.Drone_Sensors import *
#from data_classes import *

"""Run code with: python aa_mission_planner.py --Drone_ID=<Drone_ID> in Terminal!!"""

if __name__ == "__main__":

    drone_id = get_drone_id()
    print(f"Drone ID: {drone_id}")

    # INITIALIZING the necessary objects

    swarm_data = SwarmSensorData()  # Stores data for all drones
    log_data_csv = DataLogger(f"drone_{drone_id}_log.csv") 

    lora_transceiver = LoRa_Transceiver(device_id=drone_id, slot_duration=1.0)
    get_localization = WaterLinked(base_url="http://192.168.8.108", poll_interval=1.0, test_mode=False)

    #Drone Sensors

    #battery_sate = DroneSensors.read_battery_tube(0x0A)
    #print(battery_sate)

    time.sleep(0.1) #still to do -> Raniah
    position = get_localization.get_latest_position()
    print(position)
    #CALLING THE FUNCTIONS

    # Start localization
    #get_localization.start()
    #current_position = get_localization.get_latest_position()

    # Update own drone’s sensor data
    drone_sensor_data = swarm_data.drones[drone_id]  # Get the drone's entry from SwarmSensorData
    drone_sensor_data.position = position

    # Convert to list format and send via LoRa -> Implement in other file! 


    received_data = [2, 1.23, 6.56, 7.89, 10.0, 5.0, 15.0, 22.5, 1.02, 543, [2, 1, 1, 1, 1, 1, 1, 1, 0, 2]]
    swarm_data.update_from_received(received_data)
    print(f"Received Data: {received_data}")

    drone_id = received_data[0]
    if drone_id in swarm_data.drones:
        message_to_send = SensorData.format_drone_data(swarm_data.drones[drone_id])
        print(message_to_send)
    else:
        message_to_send = f"Error: Drone {drone_id} not found in swarm storage."

    # 3) Transmit the formatted data
    lora_transceiver.send_message(message_to_send)  # Uncomment in actual deployment
    print("Transmitted Data:", message_to_send)
    unpacked_message = lora_transceiver.custom_unpack(b'\x01\x00\xc8\x01\x00{\x01\x02\x90\x01\x03\x15\x01\x03\xe8\x01\x01\xf4\x01\x05\xdc\x01\x08\xca\x01\x00f\x02\x03\x00d\x00\xc8\x01,\x02\n\x00d\x00d\x00\x00\x00d\x00\x00\x00d\x00d\x00d\x00\x00\x00d')
    print(f"Received decoded message: {unpacked_message}")

    drone_id = 4
    if drone_id in swarm_data.drones:
        print(SensorData.format_drone_data(swarm_data.drones[drone_id]))
    else:
        message_to_send = f"Error: Drone {drone_id} not found in swarm storage."

    position_y = swarm_data.drones[2].waypoint
    print(position_y)




    #message_to_send = f"{drone_id}, {current_position.x}, {current_position.y}, {current_position.z}" #define specific func to do this!
    #lora_transceiver.send_message(message_to_send)
    
 

