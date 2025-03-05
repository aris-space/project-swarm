# define a base class that represents an agent and defines its state

class Robot:
    def __init__(self, name, positions):
        self.name = name
        self.positions = positions
        self.sensor_data = {}
    
    def store_sensor_data(self, sensor_type, data):
        """
        Store data for a specific sensor type.
        :param sensor_type: The type of sensor (e.g., 'pressure', 'imu').
        :param data: The sensor data to store.
        """
        self.sensor_data[sensor_type] = data

    def get_sensor_data(self, sensor_type):
        """
        Retrieve data for a specific sensor type.
        :param sensor_type: The type of sensor to retrieve.
        :return: Sensor data.
        """
        return self.sensor_data.get(sensor_type, None)
    

        