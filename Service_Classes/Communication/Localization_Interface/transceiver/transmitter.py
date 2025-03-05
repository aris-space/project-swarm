import time
import RPi.GPIO as GPIO
from LoRaRF import SX127x
from LoRaRF.base import LoRaSpi, LoRaGpio

# Setup LoRa radio
def setup_lora():
    preLeg = 5
    payLeg = 50

    # Reset GPIO pins
    #GPIO.cleanup()

    # Begin LoRa radio with connected SPI bus and IO pins (cs and reset) on GPIO
    spi = LoRaSpi(0, 0)
    cs = LoRaGpio(0, 25)
    reset = LoRaGpio(0, 24)
    LoRa = SX127x(spi, cs, reset)

    if not LoRa.begin():
        raise Exception("Can't begin LoRa radio")

    LoRa.setFrequency(169000000)  # Set frequency to 169 MHz
    LoRa.setTxPower(17, LoRa.TX_POWER_PA_BOOST)  # TX power +17 dBm using PA boost pin
    LoRa.setSpreadingFactor(7)  # Spreading factor: 7
    LoRa.setBandwidth(125000)  # Bandwidth: 125 kHz
    LoRa.setCodeRate(5)  # Coding rate: 4/5
    LoRa.setHeaderType(LoRa.HEADER_EXPLICIT)  # Explicit header mode
    #LoRa.setPreambleLength(preLeg)  # Set preamble length to 5
    LoRa.setPreambleLength(8)  # Set preamble length to 8
    LoRa.setPayloadLength(payLeg)  # Initialize payload length to 50
    LoRa.setCrcEnable(True)  # Enable CRC
    LoRa.setSyncWord(0x34)  # Set sync word to 0x34 for public network

    return LoRa

# Transmit byte data over LoRa
def transmit_data(LoRa, byte_data):
    LoRa.beginPacket()
    byte_list = list(byte_data)
    #print(byte_list)
    LoRa.write(byte_list, len(byte_list))
    LoRa.endPacket()
    LoRa.wait()

    # Print transmit time and data rate
    #print("Transmit time: {0:0.2f} ms | Data rate: {1:0.2f} byte/s".format(LoRa.transmitTime(), LoRa.dataRate()))

"""
# Main loop to transmit data
def main_transmitter(LoRa, data_gen):
    while True:
        byte_data = next(data_gen)  # Fetch next byte data to send
        transmit_data(LoRa, byte_data)
        time.sleep(0.01)  # Ensure LoRa transmission is smooth
"""