# main.py
import sys
import time
import threading

sys.path.insert(0, "/shared_folder/project-swarm")

from MainHelper.drone_instance import DroneInstance
from MainHelper.utils_functions import get_drone_id  


#---- MAIN FUNCTION
if __name__ == "__main__":



#----Initalizing DroneInstance & Drone ID

    drone_id = get_drone_id()  # Fetch the ID first
    drone_inst = DroneInstance(drone_id)  
    print(f"Drone ID: {drone_inst.drone_id}") 


#----DEFINITION OF THREADS -> calling the individual methods

    sensor_thread_obj = threading.Thread(
        target=drone_inst.sensor_thread_method,
        name="SensorThread",
        daemon=True
    )
    communication_thread_obj = threading.Thread(
        target=drone_inst.communication_thread_method,
        name="CommThread",
        daemon=True
    )
    control_thread_obj = threading.Thread(
        target=drone_inst.control_thread_method,
        name="ControlThread",
        daemon=True
    )

#----STARTING THE THREADS

    sensor_thread_obj.start()
    communication_thread_obj.start()
    control_thread_obj.start()


    while True:
        time.sleep(1)
