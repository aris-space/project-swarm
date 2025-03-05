from Communication.transceive_functions import *


if __name__ == "__main__":
    lora_transceiver = LoRa_Transceiver(device_id=1, slot_duration=1.0)

    message = 
    lora_transceiver.send_message([111.1, 21.2, 71.1])
    lora_transceiver.receive_LoRa()
    #test






