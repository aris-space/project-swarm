import os
import csv
import time

class DataLogger:
    def __init__(self, filename='loc_data_log.csv'):
        self.filename = filename
        self._initialize_file()

    def _initialize_file(self):
        """Ensure the CSV file has a header if it's new."""
        if not os.path.exists(self.filename):
            with open(self.filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Timestamp', 'Sensor', 'Data Value'])

    def log_data(self, sensor, data):
        """Log data from a specific sensor with a timestamp."""
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
        with open(self.filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, sensor, data])
        print(f"Logged data: {sensor} -> {data}")