import sys
import os
import time
import RPi.GPIO as GPIO
from Communication.LoRaRF.SX127x import SX127x
from Communication.LoRaRF.base import LoRaSpi, LoRaGpio

class LoRa_Transceiver:

    def __init__(self, device_id, slot_duration):
        self.device_id = device_id
        self.slot_duration = slot_duration
        self.string_to_transmit = ""
        self.received_message = ""
        self.request_slot = 100
        self._setup_lora()
        self.delimiter = "/"  # Message delimiter

    def send_message(self, message):
        """Set the message and transmit"""
        self.string_to_transmit = message
        self.transmit_LoRa()

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

    def process_coordinates(self, input_string):
        """Process a list of (x, y, z) coordinates."""
        triplets = [(input_string[i], input_string[i+1], input_string[i+2]) for i in range(0, len(input_string), 3)]
        return ";".join([f"{x},{y},{z}" for x, y, z in triplets])

    def transmit_LoRa(self):
        self.LoRa.purge()  # Clear buffer
        #message = self.process_coordinates(self.string_to_transmit)
        message = self.string_to_transmit
        transmit_message = (message + self.delimiter).encode('utf-8')

        transmit_start_time = time.time()
        while time.time() - transmit_start_time < self.slot_duration:
            self.LoRa.beginPacket()
            self.LoRa.write(list(transmit_message), len(transmit_message))
            self.LoRa.endPacket()
            self.LoRa.wait()
            #print("Transmit time: {0:0.2f} ms | Data rate: {1:0.2f} byte/s".format(self.LoRa.transmitTime(), self.LoRa.dataRate()))

        print(f"{self.device_id} finished transmitting: {message}.")

    def receive_LoRa(self):
        self.LoRa.purge()  # Clear previous buffer
        start_time = time.time()
        self.received_message = ""

        while time.time() - start_time < self.slot_duration:
            if not self.LoRa.request(self.request_slot):
                print("Failed to start RX mode.")
                return None

            self.LoRa.wait()
            status = self.LoRa.status()

            if status == self.LoRa.STATUS_RX_DONE:
                while self.LoRa.available() > 1:
                    self.received_message += chr(self.LoRa.read())

                print(f"Received message: {self.received_message}")
                return self.received_message

            elif status == self.LoRa.STATUS_RX_TIMEOUT:
                print("Timeout: No packet received.")
                return "No data"
            elif status == self.LoRa.STATUS_CRC_ERR:
                print("CRC Error: Corrupted packet.")
                return "Corrupted data"
            elif status == self.LoRa.STATUS_HEADER_ERR:
                print("Header Error: Invalid packet structure.")
                return "invalid packet struct"

        return None 

# Example Usage
# lora_com = LoRaCommunicator()
# lora_com.string_to_transmit = "123456789"
# lora_com.transmit_LoRa()
# lora_com.receive_LoRa()
