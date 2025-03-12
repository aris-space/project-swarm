#from Communication.transceive_functions import *
import Communication
import Localization_New
from drone_data import *

if __name__ == "__main__":


    #initialize the necessary objects
    sensor_data = SensorData()
    lora_transceiver = LoRa_Transceiver(device_id=1, slot_duration=1.0)
    localisation_blabla = # <= initialize the localization package


    #do some computation, e.g. get position from the localization package
    sensor_data.position = # <= set equal to function which gets position from the localisation package

    #send the sensor data to the ground station
    lora_transceiver.send_message(sensor_data.position)

    #receive the sensor data from the ground station
    lora_transceiver.receive_LoRa()


    #done