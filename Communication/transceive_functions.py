import sys
import os
import time
import json
import array
import struct
import RPi.GPIO as GPIO
from Communication.LoRaRF.SX127x import SX127x
from Communication.LoRaRF.base import LoRaSpi, LoRaGpio

class LoRa_Transceiver:

    def __init__(self, device_id, slot_duration):
        self.device_id = device_id  # Use self.device_id consistently
        self.slot_duration = slot_duration  # Use self.slot_duration consistently
        self.string_to_transmit = ""
        self.received_message = ""
        self.request_slot = 100
        self._setup_lora()
        self.delimiter = "/"  # Message delimiter

    def _setup_lora(self):
        preLeg = 8
        payLeg = 50

        spi = LoRaSpi(0, 0)
        cs = LoRaGpio(0, 25)
        reset = LoRaGpio(0, 24)
        self.LoRa = SX127x(spi, cs, reset)

        if not self.LoRa.begin():
            raise Exception("Can't begin LoRa radio")

        self.LoRa.setFrequency(169000000)
        self.LoRa.setTxPower(17, self.LoRa.TX_POWER_PA_BOOST)
        self.LoRa.setSpreadingFactor(7)
        self.LoRa.setBandwidth(125000)
        self.LoRa.setCodeRate(5)
        self.LoRa.setHeaderType(self.LoRa.HEADER_EXPLICIT)
        self.LoRa.setPreambleLength(preLeg)
        self.LoRa.setPayloadLength(payLeg)
        self.LoRa.setCrcEnable(True)
        self.LoRa.setSyncWord(0x34)

    # ----- DATA PACKAGER ------ 
    def custom_pack(self, data):
        """
        Packs data containing:
          - Single numbers (int/float) as marker 0x01,
          - Lists of numbers as marker 0x02,
          - Lists of state values (integers 0,1,2) as marker 0x03.
          
        Numbers are scaled by 100 and stored as 2-byte unsigned shorts.
        """
        result = bytearray()
        for item in data:
            if isinstance(item, (int, float)):
                # Marker for a single number
                result.append(0x01)
                scaled = int(float(item) * 100)
                result.extend(struct.pack(">H", scaled))
            elif isinstance(item, list):
                # First, ensure all elements are numbers (int or float)
                if all(isinstance(x, (int, float)) for x in item):
                    # If every value is one of {0, 1, 2}, pack as a state list.
                    if all(x in [0, 1, 2] for x in item):
                        result.append(0x03)      # Marker for state list
                        result.append(len(item)) # 1-byte length of sublist
                        for s in item:
                            result.append(int(s))
                    else:
                        result.append(0x02)      # Marker for list of numbers
                        result.append(len(item)) # 1-byte length of sublist
                        for num in item:
                            scaled = int(float(num) * 100)
                            result.extend(struct.pack(">H", scaled))
                else:
                    raise ValueError("Unsupported list element type")
            else:
                raise ValueError("Unsupported data type")
        return bytes(result)

    def custom_unpack(self, packed):
        """
        Unpacks data produced by custom_pack().
        Single numbers are returned as floats (scaled back by dividing by 100).
        Lists of numbers (marker 0x02) are unpacked similarly.
        Lists of state values (marker 0x03) are returned as lists of ints.
        """
        result = []
        idx = 0
        while idx < len(packed):
            marker = packed[idx]
            idx += 1
            if marker == 0x01:
                # Single number: next 2 bytes
                (scaled,) = struct.unpack(">H", packed[idx:idx+2])
                result.append(scaled / 100.0)
                idx += 2
            elif marker == 0x02:
                # List of numbers: next byte gives the length
                length = packed[idx]
                idx += 1
                sublist = []
                for _ in range(length):
                    (scaled,) = struct.unpack(">H", packed[idx:idx+2])
                    sublist.append(scaled / 100.0)
                    idx += 2
                result.append(sublist)
            elif marker == 0x03:
                # List of state values: next byte gives the length
                length = packed[idx]
                idx += 1
                state_list = []
                for _ in range(length):
                    state_list.append(packed[idx])
                    idx += 1
                result.append(state_list)
            else:
                raise ValueError("Unknown marker encountered during unpacking")
        return result

    def transmit_LoRa(self, message):
        self.LoRa.purge()  # Clear buffer
        self.LoRa.beginPacket()
        self.LoRa.put(message)
        self.LoRa.endPacket()
        self.LoRa.wait()

        print(f"{self.device_id} finished transmitting: {message}")
        print("Transmit time: {0:0.2f} ms | Data rate: {1:0.2f} byte/s".format(self.LoRa.transmitTime(), self.LoRa.dataRate()))

    def receive_LoRa(self):
        self.LoRa.purge()  # Clear previous buffer

        if not self.LoRa.request(self.request_slot):
            print("Failed to start RX mode.")
            return None

        self.LoRa.wait()
        status = self.LoRa.status()

        if status == self.LoRa.STATUS_RX_DONE:
            received_bytes = b""
            while self.LoRa.available() > 1:
                received_bytes += self.LoRa.read().to_bytes(1, 'big')
            decoded_data = self.custom_unpack(received_bytes)
            print(f"Received message: {decoded_data}")
            return decoded_data

        elif status == self.LoRa.STATUS_RX_TIMEOUT:
            print("Timeout: No packet received.")
            return None
        elif status == self.LoRa.STATUS_CRC_ERR:
            print("CRC Error: Corrupted packet.")
            return None
        elif status == self.LoRa.STATUS_HEADER_ERR:
            print("Header Error: Invalid packet structure.")
            return None

        return None 

    def main_locom(self, message):
        """
        Function to handle the main communication loop of the LoRa transceiver.
        Uses self.device_id and self.slot_duration.
        """
        # Use self.device_id for identification
        is_transmitting = (self.device_id == 1)
        matching_message = None

        print(f"{self.device_id} initialized. Starting communication protocol...")

        # TRANSMITTING
        while True: 
            if is_transmitting:

                if self.device_id == 1:
                    print(f"{self.device_id} transmitting: {message}")
                    transmit_message = self.custom_pack(message)
                    transmit_start_time = time.time()
                    while time.time() - transmit_start_time < self.slot_duration:
                        self.transmit_LoRa(transmit_message)
                    is_transmitting = False  # Moved outside the while loop

                elif self.device_id == 2:
                    # For device 2, ensure matching_message is set before transmitting
                    print(f"{self.device_id} is transmitting: {matching_message}")
                    if matching_message is None:
                        print("No matching message to transmit for device 2.")
                        return None
                    transmit_message = self.custom_pack(matching_message)
                    transmit_start_time = time.time()
                    while time.time() - transmit_start_time < self.slot_duration:
                        self.transmit_LoRa(transmit_message)
                    is_transmitting = False  # Moved outside the while loop

            # RECEIVING
            else:
                if self.device_id == 2:
                    print(f"{self.device_id} receiving for {self.slot_duration} seconds...")
                    start_time = time.time()
                    while time.time() - start_time < self.slot_duration:
                        msg = self.receive_LoRa()
                        if msg is not None:
                            # A real message was received; break out early.
                            if msg == "Acknowledged":
                                return "Achnowledged" and False
                            if msg == "Non-Matching":
                                return "Non-Matching" and False
                            is_transmitting = True
                            matching_message = msg
                            return msg
                    matching_message = [99]
                    return "No data"

                elif self.device_id == 1:
                    print(f"{self.device_id} receiving for {self.slot_duration} seconds...")
                    start_time = time.time()
                    while time.time() - start_time < self.slot_duration:
                        msg = self.receive_LoRa()
                        if msg is not None:
                            # A real message was received; break out early.
                            if msg == message: 
                                return "Acknowledged" and False
                            else: 
                                return "Non-matching" and False
                            
                    is_transmitting = True
                    return "No data"



                """
                Write into main_locom_pool
                if matching_message:
                    response = "Acknowledged" if matching_message == random_sentence else "Non-Matching"
                    print(f"{self.device_id} message validation: {response}")

                    byte_list = encode_sentence(response)
                    message = byte_list + [ord(DELIMITER)]
                    transmitter_func(self.device_id, SLOT_DURATION, message, lora_tx)
                    print(f"{self.device_id} finished transmitting. END OF PROGRAM")
                else:
                    print(f"{self.device_id} did not receive valid data.")
                break"
                """


