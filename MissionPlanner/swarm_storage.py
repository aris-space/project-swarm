from dataclasses import dataclass, field
from typing import Optional, List, Dict
from datetime import datetime

@dataclass
class Position:
    x: float = 99.0
    y: float = 99.0
    z: float = 99.0

@dataclass
class Orientation:
    roll: float = 99.0
    pitch: float = 99.0
    yaw: float = 99.0

@dataclass
class GeneralSystemState:
    temperature: float = 99.0
    conductivity: float = 99.0

@dataclass
class SensorData:
    """Stores sensor data for a single drone, including last received data."""
    drone_id: int
    position: Position = field(default_factory=Position)
    orientation: Orientation = field(default_factory=Orientation)
    system_state: GeneralSystemState = field(default_factory=GeneralSystemState)
    waypoint: int = 99  # Single waypoint integer (expected range 1-1000)
    states: List[int] = field(default_factory=lambda: [2]*10)  # Default: list of ten 2's
    battery_data: Dict[str, float] = field(default_factory=lambda: {"temperature": None, "humidity": None, "Bat_temperature": None})

    last_data: Optional["SensorData"] = None  # Stores the last valid data
    last_received_time: Optional[datetime] = None  # Timestamp of last update

    def update_from_list(self, data: List):
        """
        Updates the sensor data from a received list.
        Supports two formats:
          - Full update: [drone_id, x, y, z, roll, pitch, yaw, temperature, conductivity, waypoint, states]
          - Position-only update: [x, y, z] (keeps other values at their defaults)
        """
        try:
            # Backup current values
            self.last_data = SensorData(
                self.drone_id,
                position=self.position,
                orientation=self.orientation,
                system_state=self.system_state,
                waypoint=self.waypoint,
                states=self.states.copy(),
                last_received_time=self.last_received_time
            )
            if len(data) == 3:
                # Localization update: only position [x, y, z]
                self.position = Position(x=data[0], y=data[1], z=data[2])
            else:
                # Full update: first element is drone_id, next three are position, etc.
                self.position = Position(x=data[1], y=data[2], z=data[3])
                self.orientation = Orientation(roll=data[4], pitch=data[5], yaw=data[6])
                self.system_state = GeneralSystemState(temperature=data[7], conductivity=data[8])
                self.waypoint = data[9] if isinstance(data[9], int) else 99

                if len(data) > 10 and isinstance(data[10], list):
                    received_states = data[10][:10]
                    self.states = [int(x) for x in received_states] + [2] * (10 - len(received_states))
                else:
                    self.states = [2] * 10

            self.last_received_time = datetime.now()
        except IndexError:
            print(f"Warning: Incomplete data received for Drone {self.drone_id}")

    def format_drone_data(self) -> list:
        """
        Formats the sensor data of a single drone into a list suitable for transmission.
        Returns:
          [drone_id, x, y, z, roll, pitch, yaw, temperature, conductivity, waypoint, states]
        """
        return [
            self.drone_id,
            self.position.x,
            self.position.y,
            self.position.z,
            self.orientation.roll,
            self.orientation.pitch,
            self.orientation.yaw,
            self.system_state.temperature,
            self.system_state.conductivity,
            self.waypoint,
            self.states
        ]

    # New property getters for easy access to position coordinates
    @property
    def x(self):
        return self.position.x

    @property
    def y(self):
        return self.position.y

    @property
    def z(self):
        return self.position.z


@dataclass
class SwarmSensorData:
    """Manages sensor data for the entire swarm of 4 drones."""
    drones: Dict[int, SensorData] = field(default_factory=lambda: {i: SensorData(i) for i in range(1, 5)})

    def update_from_received(self, received_data):
        """
        Parses received LoRa data and updates the corresponding drone’s sensor data.
        Expects received_data to be either a string or a list in the format:
          [drone_id, x, y, z, roll, pitch, yaw, temperature, conductivity, waypoint, states]
        where numeric defaults are 99 and the states list is made up of ints.
        """
        if received_data == "No data":
            print("No data received, keeping previous values.")
            return
        elif received_data in ["Corrupted data", "invalid packet struct"]:
            print("Received corrupted packet, resetting affected drone's data to default values.")
            try:
                drone_id = int(received_data.split(',')[0]) if received_data[0].isdigit() else None
                if drone_id and drone_id in self.drones:
                    self.drones[drone_id] = SensorData(drone_id)
            except Exception as e:
                print(f"Error during reset: {e}")
            return
        
        try:
            data_list = eval(received_data) if isinstance(received_data, str) else received_data
            drone_id = int(data_list[0])
            if drone_id in self.drones:
                self.drones[drone_id].update_from_list(data_list)
        except Exception as e:
            print(f"Error parsing received data: {e}")


"""EXAMPLE USAGE"""

""" #Receive and update swarm data
    #received_data = lora_transceiver.receive_LoRa()
    received_data = [2, 1.23, 6.56, 7.89, 10.0, 5.0, 15.0, 22.5, 1.02, [1, 2, 3], [True, True, False, True, False, True, True, True, False, True]]
    swarm_data.update_from_received(received_data)
    print(f"Received Data: {received_data}")

    position_y = swarm_data.drones[2].position.y

    received_data = [2, 1.23, 4.56, 7.89, 10.0, 5.0, 15.0, 22.5, 1.02, [1, 2, 3], [True, True, False, True, False, True, True, True, False, True]]
    swarm_data.update_from_received(received_data)
    print(f"Received Data: {received_data}")
    
    previous_position_y = swarm_data.drones[2].last_data.position.y 
    print(previous_position_y)"""