#example usage: Controls Extract Position

#position_y = swarm_data.drones[drone_id].position.y
#print(position_y)

#position_of_drone_1 = swarm_data.drones[drone_id].position
#print("Complete Position:", position_of_drone_1) -> Position(x=71.21, y=62.1, z=76.21)

from dataclasses import dataclass, field
from typing import Optional, List, Dict
from datetime import datetime
import numpy as np

# --- Common Structures ---
@dataclass
class Position:
    x: float = 99.0
    y: float = 99.0
    z: float = 99.0

# Removed Orientation as per instructions

@dataclass
class Acceleration:
    ax: float
    ay: float
    az: float

@dataclass
class AngularRates:
    roll_rate: float
    pitch_rate: float
    yaw_rate: float

@dataclass
class Quaternion:
    qx: float
    qy: float
    qz: float
    qw: float

# --- Raw Sensor Data Class ---
@dataclass
class DroneRawData:
    drone_id: int = 99
    position: Position = field(default_factory=Position)
    # Orientation removed
    temperature: float = 99.0
    conductivity: float = 99.0
    sensor_depth: float = 99.0
    battery_data: Dict[str, float] = field(default_factory=lambda: {"temperature": None, "humidity": None, "Bat_temperature": None})
    last_data: Optional["DroneRawData"] = None
    last_received_time: Optional[datetime] = None

    def update_from_list(self, data: List):
        try:
            # Backup current values
            self.last_data = DroneRawData(
                drone_id=self.drone_id,
                position=self.position,
                temperature=self.temperature,
                conductivity=self.conductivity,
                sensor_depth=self.sensor_depth,
                battery_data=self.battery_data.copy(),
                last_received_time=self.last_received_time
            )
            if len(data) == 3:
                # Localization update: only position [x, y, z]
                self.position = Position(x=data[0], y=data[1], z=data[2])
            else:
                # Full update: assume data format is now [drone_id, x, y, z, temperature, conductivity, ...]
                self.position = Position(x=data[1], y=data[2], z=data[3])
                self.temperature = data[4]
                self.conductivity = data[5]
            self.last_received_time = datetime.now()
        except IndexError:
            print(f"Warning: Incomplete raw data received for Drone {self.drone_id}")

    def format_drone_data(self) -> list:
        return [
            self.drone_id,
            self.position.x,
            self.position.y,
            self.position.z,
            self.temperature,
            self.conductivity,
        ]

    # --- Update methods for raw sensor values ---
    def update_position(self, *, x: Optional[float] = None, y: Optional[float] = None, z: Optional[float] = None, values: Optional[List[float]] = None):
        if values is not None:
            if len(values) != 3:
                raise ValueError("A list of exactly three values is required for position (x, y, z).")
            self.position.x, self.position.y, self.position.z = values
        else:
            if x is not None:
                self.position.x = x
            if y is not None:
                self.position.y = y
            if z is not None:
                self.position.z = z
        self.last_received_time = datetime.now()

    def update_temperature(self, new_temperature: float):
        self.temperature = new_temperature
        self.last_received_time = datetime.now()

    def update_conductivity(self, new_conductivity: float):
        self.conductivity = new_conductivity
        self.last_received_time = datetime.now()

    def update_sensor_depth(self, depth: float):
        self.sensor_depth = depth
        self.last_received_time = datetime.now()

    def update_battery_data(self, new_battery_data: Dict[str, float]):
        self.battery_data.update(new_battery_data)
        self.last_received_time = datetime.now()


# --- Filtered Sensor Data Class ---
@dataclass
class DroneFilteredData:
    drone_id: int = 99
    position: Position = field(default_factory=Position)
    # Orientation removed
    temperature: float = 99.0
    conductivity: float = 99.0
    sensor_depth: float = 99.0
    battery_data: Dict[str, float] = field(default_factory=lambda: {"temperature": None, "humidity": None, "Bat_temperature": None})
    # Only in filtered data:
    waypoint: int = 99
    states: List[int] = field(default_factory=lambda: [2]*10)
    # New sensor fields (only in filtered data)
    acceleration: Optional[Acceleration] = None
    angular_rates: Optional[AngularRates] = None
    quaternion: Optional[Quaternion] = None

    last_data: Optional["DroneFilteredData"] = None
    last_received_time: Optional[datetime] = None

    def format_drone_data(self) -> list:
        return [
            self.drone_id,
            self.position.x,
            self.position.y,
            self.position.z,
            self.temperature,
            self.conductivity,
            self.waypoint,
            self.states
        ]

    # --- Update methods for filtered sensor values ---
    def update_position(self, *, x: Optional[float] = None, y: Optional[float] = None, z: Optional[float] = None, values: Optional[List[float]] = None):
        if values is not None:
            if len(values) != 3:
                raise ValueError("A list of exactly three values is required for position (x, y, z).")
            self.position.x, self.position.y, self.position.z = values
        else:
            if x is not None:
                self.position.x = x
            if y is not None:
                self.position.y = y
            if z is not None:
                self.position.z = z
        self.last_received_time = datetime.now()

    def update_temperature(self, new_temperature: float):
        self.temperature = new_temperature
        self.last_received_time = datetime.now()

    def update_conductivity(self, new_conductivity: float):
        self.conductivity = new_conductivity
        self.last_received_time = datetime.now()

    def update_sensor_depth(self, depth: float):
        self.sensor_depth = depth
        self.last_received_time = datetime.now()

    def update_battery_data(self, new_battery_data: Dict[str, float]):
        self.battery_data.update(new_battery_data)
        self.last_received_time = datetime.now()

    def update_waypoint(self, new_waypoint: int):
        self.waypoint = new_waypoint
        self.last_received_time = datetime.now()

    def update_states(self, new_states: List[int]):
        self.states = new_states[:10] + [2] * (10 - len(new_states))
        self.last_received_time = datetime.now()

    # --- New Update Methods for Additional Sensors ---
    def update_acceleration(self, values: np.ndarray):
        if len(values) != 3:
            raise ValueError("A numpy array of exactly three values is required for acceleration (ax, ay, az).")
        self.acceleration = Acceleration(ax=float(values[0]), ay=float(values[1]), az=float(values[2]))
        self.last_received_time = datetime.now()

    def update_angular_rates(self, values: np.ndarray):
        if len(values) != 3:
            raise ValueError("A numpy array of exactly three values is required for angular rates (roll_rate, pitch_rate, yaw_rate).")
        self.angular_rates = AngularRates(roll_rate=float(values[0]), pitch_rate=float(values[1]), yaw_rate=float(values[2]))
        self.last_received_time = datetime.now()

    def update_quaternion(self, values: np.ndarray):
        if len(values) != 4:
            raise ValueError("A numpy array of exactly four values is required for quaternion (qx, qy, qz, qw).")
        self.quaternion = Quaternion(qx=float(values[0]), qy=float(values[1]), qz=float(values[2]), qw=float(values[3]))
        self.last_received_time = datetime.now()


# --- Swarm Storage ---
@dataclass
class SwarmSensorData:
    """
    Manages sensor data for the entire swarm.
    Maintains two dictionaries:
      - raw_drones: updated directly via received sensor data.
      - filtered_drones: holds filtered values (including waypoint, states, and additional sensor data).
    """
    raw_drones: Dict[int, DroneRawData] = field(default_factory=lambda: {i: DroneRawData(i) for i in range(1, 5)})
    filtered_drones: Dict[int, DroneFilteredData] = field(default_factory=lambda: {i: DroneFilteredData(i) for i in range(1, 5)})

    def update_from_received(self, received_data):
        """
        Parses received LoRa data and updates the corresponding drone’s raw sensor data.
        Expects received_data to be either None or a list in the format:
          [drone_id, x, y, z, temperature, conductivity, ...]
        If received_data is None, the update is skipped.
        """
        if received_data is None:
            return
        try:
            data_list = received_data if isinstance(received_data, list) else eval(received_data)
            drone_id = int(data_list[0])
            if drone_id in self.raw_drones:
                self.raw_drones[drone_id].update_from_list(data_list)
        except Exception as e:
            print(f"Error parsing received data: {e}")

    @property
    def position(self) -> Optional[Position]:
        return self._position

    @position.setter
    def position(self, value: Optional[Position]) -> None:
        self._position = value
