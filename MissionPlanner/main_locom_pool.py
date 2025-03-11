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

    #GET LOCALIZATION: 

    #position = get_localization.get_latest_position(timeout = 2)
    #print(position)
    # New localization data: only the current position [x, y, z]
    position_data = [71.21, 62.10, 76.21]

    # Update drone 1's sensor data with the new position.
    # This call will only update the position while keeping other values at their defaults.
    swarm_data.drones[drone_id].update_from_list(position_data)

    # Extracting the complete Position object for drone 1:
    position_of_drone_1 = swarm_data.drones[drone_id].position
    print("Complete Position:", position_of_drone_1)

    # Format the drone data for transmission
    message_to_send = swarm_data.drones[drone_id].format_drone_data()
    lora_transceiver.main_locom(message_to_send)

    


    #COMMUNICATION LOOP

    #lora_transceiver()




