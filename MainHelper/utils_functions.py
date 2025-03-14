import getopt
import sys
import csv

# ========== Drone ID Initializer ==========

def get_drone_id():
    """Parses command-line arguments to retrieve the Drone ID."""
    argumentList = sys.argv[1:]
    options = "hd:o:"
    long_options = ["Help", "Drone_ID=", "Output="]
    
    drone_id = None  # Default if no ID is provided

    try:
        # Parse the arguments
        arguments, values = getopt.getopt(argumentList, options, long_options)
        
        for currentArgument, currentValue in arguments:
            if currentArgument in ("-h", "--Help"):
                print("Usage: python aa_mission_planner.py --Drone_ID=<id>")
                sys.exit(0)  # Exit after displaying help
            elif currentArgument in ("-d", "--Drone_ID"):
                drone_id = int(currentValue)  # Convert ID to integer
            elif currentArgument in ("-o", "--Output"):
                print(f"Enabling special output mode ({currentValue})")
    
    except getopt.error as err:
        print(f"Argument error: {err}")
        sys.exit(1)  # Exit on error

    if drone_id is None:
        print("Error: No Drone ID provided. Use --Drone_ID=<id>")
        sys.exit(1)

    return drone_id

# ========== Data Logger ==========
class DataLogger:
    def __init__(self, filename: str):
        self.filename = filename
        with open(self.filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                "drone_id", "x", "y", "z",
                "roll", "pitch", "yaw",
                "temp_sensor", "conductivity_sensor", "waypoints",
                *[f"state{i+1}" for i in range(10)]
            ])

    def log_data(self, sensor_data):
        """Append sensor data to the CSV log file."""
        with open(self.filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(sensor_data.to_list())

# ========== Put Data into List (to transmit) ==========
