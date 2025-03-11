import smbus2
import time
import struct
import pigpio

class BatterySensors:
    
    BATTERY_TUBE_A_ADDR = 0x0A  # Arduino Nano A
    BATTERY_TUBE_B_ADDR = 0x0B  # Arduino Nano B

    def __init__(self, i2c_bus=1):
        # Initialize I2C bus for Raspberry Pi (or similar)
        self.bus = smbus2.SMBus(i2c_bus)
        # Initialize battery tube data dictionaries
        self.battery_tube_A = {"temperature": None, "humidity": None, "Bat_temperature": None}
        self.battery_tube_B = {"temperature": None, "humidity": None, "Bat_temperature": None}

    def read_battery_tube(self, address):
        """Reads temperature, humidity, and battery temperature from the specified battery tube."""
        try:
            # Request 12 bytes (3 floats, 4 bytes each)
            data = self.bus.read_i2c_block_data(address, 0, 12)
            # Unpack little-endian floats from the data
            temperature = struct.unpack('<f', bytes(data[:4]))[0]
            humidity = struct.unpack('<f', bytes(data[4:8]))[0]
            batTemp = struct.unpack('<f', bytes(data[8:]))[0]
            return temperature, humidity, batTemp
        except Exception as e:
            print(f"Error reading from Battery Tube {hex(address)}: {e}")
            return None, None, None

    def update_battery_status(self):
        """Updates the battery tube dictionaries with the latest sensor readings."""
        # Update Battery Tube A
        temp_A, hum_A, batTemp_A = self.read_battery_tube(self.BATTERY_TUBE_A_ADDR)
        if temp_A is not None and hum_A is not None and batTemp_A is not None:
            self.battery_tube_A["temperature"] = temp_A
            self.battery_tube_A["humidity"] = hum_A
            self.battery_tube_A["Bat_temperature"] = batTemp_A

        # Update Battery Tube B
        temp_B, hum_B, batTemp_B = self.read_battery_tube(self.BATTERY_TUBE_B_ADDR)
        if temp_B is not None and hum_B is not None and batTemp_B is not None:
            self.battery_tube_B["temperature"] = temp_B
            self.battery_tube_B["humidity"] = hum_B
            self.battery_tube_B["Bat_temperature"] = batTemp_B

    def battery_states(self):
        """
        Returns the current battery states as a dictionary.
        This method first updates the battery readings.
        """
        self.update_battery_status()
        return {
            "Battery Tube A": self.battery_tube_A,
            "Battery Tube B": self.battery_tube_B
        }

    def battery_tube_readouts(self, interval=2):
        """
        Continuously prints the battery tube statuses at the specified interval (in seconds).
        Use this method for debugging or logging.
        """
        while True:
            self.update_battery_status()
            print(
                f"Battery Tube A - Temp: {self.battery_tube_A['temperature']}°C, "
                f"Humidity: {self.battery_tube_A['humidity']}%, "
                f"Bat_temperature: {self.battery_tube_A['Bat_temperature']}°C"
            )
            print(
                f"Battery Tube B - Temp: {self.battery_tube_B['temperature']}°C, "
                f"Humidity: {self.battery_tube_B['humidity']}%, "
                f"Bat_temperature: {self.battery_tube_B['Bat_temperature']}°C\n"
            )
            time.sleep(interval)


class TemperaturePressure:
    
    def __init__(self, i2c_bus=1, sensor_address=0x40):
        self.pi = pigpio.pi()  # Connect to the pigpio daemon
        self.handle = self.pi.i2c_open(i2c_bus, sensor_address)  # Open I2C
        self.zero_air_pressure = None  # Initial air pressure reference

    def crc8(self, data):
        crc = 0
        for d in data:
            crc ^= d << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc ^= 0x8380
                crc <<= 1
        return (crc >> 8) & 0xFF

    def read_sensor_float(self, regAddr):
        request = [0x20, regAddr]
        request.append(self.crc8(request))
        self.pi.i2c_write_device(self.handle, bytes(request))
        time.sleep(0.02)  # Wait for sensor data
        count, data = self.pi.i2c_read_device(self.handle, 7)
        if count < 7:
            print("\n\n\n>>>>>>>> ERROR: read 7 bytes\n\n\n")
            return None
        response = list(data)
        if self.crc8(response[:6]) != response[6]:
            print("\n\n\n>>>>>>>> ERROR: CRC\n\n\n")
            return None
        if (response[0] & 0x10) == 0:
            print("\n\n\n>>>>>>>> ERROR: ready flag\n\n\n")
            return None
        raw = (response[2] << 24) | (response[3] << 16) | (response[4] << 8) | response[5]
        return struct.unpack("!f", struct.pack("!I", raw))[0]

    def get_depth(self):
        g = 9.80600  # Gravity [m/s^2]
        density = 997.0474  # Freshwater density [kg/m^3]
        pressure = self.read_sensor_float(0x00)  # Read pressure in bar
        if pressure is None:
            return None
        if self.zero_air_pressure is None:
            self.zero_air_pressure = pressure
        #print("P=", pressure, "bar")
        depth = (pressure - self.zero_air_pressure) * 100000 / (g * density)
        return round(depth, 3)

    def get_temperature(self):
        temp = self.read_sensor_float(0x04)
        if temp is None:
            return None
        return round(temp, 1)

    def close(self):
        self.pi.i2c_close(self.handle)
        self.pi.stop()
 
