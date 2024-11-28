# import time
# import board
# import adafruit_dht
# import psutil
# # We first check if a libgpiod process is running. If yes, we kill it!
# for proc in psutil.process_iter():
#     if proc.name() == 'libgpiod_pulsein' or proc.name() == 'libgpiod_pulsei':
#         proc.kill()
# sensor = adafruit_dht.DHT11(board.D23)
# while True:
#     try:
#         temp = sensor.temperature
#         humidity = sensor.humidity
#         print("Temperature: {}*C   Humidity: {}% ".format(temp, humidity))
#     except RuntimeError as error:
#         print(error.args[0])
#         time.sleep(2.0)
#         continue
#     except Exception as error:
#         sensor.exit()
#         raise error
#     time.sleep(2.0)



import time
import board
import adafruit_dht
import psutil
# We first check if a libgpiod process is running. If yes, we kill it!



class Humidity_Sensor:
    
    def __init__(self): 
        # Iterate through all running processes on the system
        for proc in psutil.process_iter():
            try:
                # Check if the process name matches 'libgpiod_pulsein' or 'libgpiod_pulsei'
                # These are processes used by the DHT library to interface with GPIO pins
                if proc.name() == 'libgpiod_pulsein' or proc.name() == 'libgpiod_pulsei':
                    # Kill the process to avoid conflicts or ensure a clean state for GPIO access
                    proc.kill()
            except Exception as e:
                # Handle any errors that might occur while iterating or killing the process
                print(f"Could not terminate process {proc.name()}: {e}")

        # Initialize the DHT11 sensor using the Adafruit library
        # 'board.D23' specifies the GPIO pin the sensor is connected to (pin 23 in this case)
        self.sensor = adafruit_dht.DHT11(board.D23)

        
    def read_data(self):
        try:
            temp = self.sensor.temperature
            humidity = self.sensor.humidity
            print("Temperature: {}*C   Humidity: {}% ".format(temp, humidity))
        except RuntimeError as error:
            print(error.args[0])
            time.sleep(2.0)

        except Exception as error:
            self.sensor.exit()
            raise error
