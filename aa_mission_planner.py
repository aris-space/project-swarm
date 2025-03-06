from Communication.transceive_functions import *
from Utils.utils_functions import *
from Localization_New.Waterlinked.waterlinked import *

if __name__ == "__main__":
    lora_transceiver = LoRa_Transceiver(device_id=1, slot_duration=4.0)
    logger = DataLogger()

    #lora_transceiver.send_message([111.1, 21.2, 71.1])
    lora_transceiver.receive_LoRa()
    logger.log_data("GPS", [47.3769, 8.5417])







