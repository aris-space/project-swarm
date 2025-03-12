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

    lora_transceiver = LoRa_Transceiver(device_id=drone_id, slot_duration=4)
    get_localization = WaterLinked(base_url="http://192.168.8.108", poll_interval=1.0, test_mode=False)

    Keller_Sensor = TemperaturePressure() #need sudo pigpiod!!!!


    #GET LOCALIZATION: 

    #position_data = get_localization.get_latest_position(timeout = 2)
    #print(position_data)
    # New localization data: only the current position [x, y, z]
    position_data = [71.21, 62.10, 76.21]

    temperature = Keller_Sensor.get_temperature()
    swarm_data.drones[drone_id].update_temperature(temperature)
    temperature_of_current_drone = swarm_data.drones[drone_id].system_state.temperature
    print(temperature_of_current_drone)



    """
    start_time = time.time()
    while time.time() - start_time < 5:
        depth = Keller_Sensor.get_depth()
        print(depth)
    
    #swarm_data.drones[drone_id].update_position(z=depth)
    #depth_of_current_drone = swarm_data.drones[drone_id].z
    #print(depth_of_current_drone)
    
    # Update drone 1's sensor data with the new position.
    # This call will only update the position while keeping other values at their defaults.
    swarm_data.drones[drone_id].update_from_list(position_data)

    # Extracting the complete Position object for drone 1:
    position_of_drone_1 = swarm_data.drones[drone_id].position
    print("Complete Position:", position_of_drone_1)
    position_y = swarm_data.drones[drone_id].position.y
    print(position_y)

    # Format the drone data for transmission
    message_to_send = swarm_data.drones[drone_id].format_drone_data()
    received_list = lora_transceiver.main_locom(message_to_send)

    if received_list == message_to_send:
        print("Acknowledged") 
    else: 
        print("Non-matching")
    """
    

    


    #COMMUNICATION LOOP

    #lora_transceiver()




