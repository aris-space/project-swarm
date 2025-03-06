import sys
import os
import csv #for log
import time
import random
import string

# Add the 'lora' directory to the sys.path
sys.path.append(os.path.join(os.path.dirname(__file__), 'transceiver'))

# Import modules after adding the path
from transmitter import setup_lora, transmit_data
from receiver import setup_lora as setup_receiver

print("test")

lora_tx = setup_lora()
lora_rx = setup_receiver()


DEVICE_ID = "Device_1"  # Change to "Device_2" for SUBMERGED ONE
SLOT_DURATION = 1  # Duration for transmit/receive slots (in seconds)
DELIMITER = "\n"  # Message delimiter 


coordinates = [1.243, 76.2222, 21.32] 

def process_coordinates(coordinates):
    """Process a list of (x, y, z) coordinates coming from an external source."""
    
    # Convert the flat list into (x, y, z) tuples
    triplets = [(coordinates[i], coordinates[i+1], coordinates[i+2]) for i in range(0, len(coordinates), 3)]
    
    # Convert triplets into formatted strings
    return ";".join([f"{x},{y},{z}" for x, y, z in triplets])

def transmitter_func(DEVICE_ID, SLOT_DURATION, message, lora_tx):
    lora_tx.purge() # Clear buffer
    transmit_start_time = time.time()

    go_message = (message + DELIMITER).encode('utf-8')  # Encode string as bytes
    print(f"{DEVICE_ID} transmitting: {go_message}")

    while time.time() - transmit_start_time < SLOT_DURATION:
        transmit_data(lora_tx, go_message)
    print(f"{DEVICE_ID} finished transmitting. Switching to receiving mode...")

    return

def receiver_func(SLOT_DURATION, lora_rx):
    """
    Listens for an incoming LoRa packet for SLOT_DURATION (approx. 3 seconds).
    If a packet is received, it decodes and prints it.
    """

    lora_rx.purge()  # Clear previous buffer
    start_time = time.time()
    print(f"Listening for up to {SLOT_DURATION} seconds...")


    #while time.time() - start_time < SLOT_DURATION:
    while True:
    #time.time() - start_time < SLOT_DURATION:
        # Request to receive for 3000 ms (3 seconds)
        if not lora_rx.request(100):  # Non-blocking RX mode
            print("Failed to start RX mode.")
            return None

        # Wait for the packet to be received
        lora_rx.wait()  # Blocks until RX is done

        # Check if reception was successful
        status = lora_rx.status()
        if status == lora_rx.STATUS_RX_DONE:
            message = ""
            while lora_rx.available() > 1:
                message += chr(lora_rx.read())  # Convert received bytes to string
            counter = lora_rx.read()  # Last byte may be a counter (ignore if not used)


            print(f"Received message: {message}")
            return message  # Exit immediately if message found

        # Handle errors or timeout
        elif status == lora_rx.STATUS_RX_TIMEOUT:
            print("Timeout: No packet received.")

        elif status == lora_rx.STATUS_CRC_ERR:
            print("CRC Error: Corrupted packet.")

        elif status == lora_rx.STATUS_HEADER_ERR:
            print("Header Error: Invalid packet structure.")




def log_data_to_csv(log_data = False):
    # Check if the CSV file already exists
    file_exists = os.path.exists('loc_data_log.csv')

    # Open the CSV file in append mode
    with open('loc_data_log.csv', mode='a', newline='') as file:
        writer = csv.writer(file)

        # If the file doesn't exist, write the header
        if not file_exists:
            writer.writerow(['Timestamp', 'Data Value'])

        # Replace this with your actual communication code to get the value
        data_value = log_data  
        
        # Get the current timestamp
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
        
        # Write the timestamp and data value to the CSV
        writer.writerow([timestamp, data_value])

def main():
    # Setup LoRa transmitter and receiver
    global lora_rx
    global lora_tx
    global message_received 

    # Determine start mode (Device 1 starts transmitting; Device 2 starts receiving)
    is_transmitting = DEVICE_ID == "Device_1"
    transmit_data = None 
    matching_message = None
    response = None

    print(f"{DEVICE_ID} initialized. Starting communication protocol...")

    while True:
        if is_transmitting:

            print("is_transmitting now")

            if DEVICE_ID == "Device_1":
                # Device 1: Transmit random sentence every 1 second for 5 seconds
                #random_sentence = generate_random_sentence()
                transmit_data = process_coordinates(coordinates)

                transmitter_func(DEVICE_ID, SLOT_DURATION, transmit_data, lora_tx)
                is_transmitting = False  # Switch to receiving mode

            elif DEVICE_ID == "Device_2":
                #if random_sentence is None:
                #    random_sentence = matching_message  # Use received message for transmission
                #if random_sentence is None:
                #    print("Error: No message to transmit.")
                #    break  # Exit loop if no valid message

                transmitter_func(DEVICE_ID, SLOT_DURATION, matching_message, lora_tx)
                is_transmitting = False  # Switch to receiving mode
                matching_message = False

        else:
            if DEVICE_ID == "Device_1":
                print(f"{DEVICE_ID} receiving for {SLOT_DURATION} seconds...")
                byte_data = receiver_func(SLOT_DURATION, lora_rx)
                #print(f"Raw received data (hex): {byte_data.hex()}")
                print(f"{DEVICE_ID} received {byte_data}")
                message = byte_data
                #if DELIMITER.encode('utf-8') in byte_data:
                # Decode message
                try:
                    #message = byte_data.decode('utf-8')
                    #print(f"{DEVICE_ID} received: {message}")

                    #matching_message = message.strip(DELIMITER)  # Remove delimiter
                    #print(f"{DEVICE_ID} received unstripped message: {matching_message}")

                    if message == transmit_data:
                        response = "Acknowledged"
                    else:
                        response = "Non-Matching"
                    print(f"{DEVICE_ID} message validation: {response}")

                    transmitter_func(DEVICE_ID, SLOT_DURATION, response, lora_tx)
                    #print(f"{DEVICE_ID} finished transmitting. END OF PROGRAM")
                    is_transmitting = True

                except UnicodeDecodeError as e:
                    print(f"Failed to decode received data: {e}")
                    break

            elif DEVICE_ID == "Device_2":
                print(f"{DEVICE_ID} receiving for {SLOT_DURATION} seconds...")

                byte_data = receiver_func(SLOT_DURATION, lora_rx)
                #print(f"Raw received data (hex): {byte_data.hex()}")
                print(f"{DEVICE_ID} received {byte_data}")
                matching_message = byte_data
                log_data = byte_data
                log_data_to_csv(log_data)
                is_transmitting = True


if __name__ == "__main__":
    main()