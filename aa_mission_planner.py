from Communication.transceive_functions import *
from Localization_New.Waterlinked.waterlinked import *
from Utils.utils_functions import *


if __name__ == "__main__":
    lora_transceiver = LoRa_Transceiver(device_id=1, slot_duration=1.0)
    get_position = WaterLinked(base_url="http://192.168.7.1", poll_interval=1.0, test_mode=True)
    log_data_csv = DataLogger("test-log.csv")

    get_position.start()
    current_position = get_position.get_latest_position()
    lora_transceiver.send_message(current_position)
    log_data = lora_transceiver.receive_LoRa()
    log_data_csv.log_data("Localization ", log_data)






