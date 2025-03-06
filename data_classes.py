from dataclasses import dataclass

@dataclass
class Position:
    x: float
    y: float
    z: float

@dataclass
class Velocity:
    vx: float
    vy: float
    vz: float

@dataclass
class Acceleration:
    ax: float
    ay: float
    az: float

@dataclass
class Orientation:
    roll: float
    pitch: float
    yaw: float

@dataclass
class AngularRates:
    roll_rate: float
    pitch_rate: float
    yaw_rate: float

@dataclass
class AngularAcceleration:
    roll_acc: float
    pitch_acc: float
    yaw_acc: float

@dataclass
class GeneralSystemState:
    battery_voltage: float = 0.0
    battery_current: float = 0.0
    percentage: float = 0.0
    pressure: float = 0.0
    temperature: float = 0.0

# ========== Core Data Classes ==========
class SensorData:
    def __init__(
        self, 
        position: Optional[Position] = None,
        velocity: Optional[Velocity] = None,
        acceleration: Optional[Acceleration] = None,
        orientation: Optional[Orientation] = None,
        angular_rates: Optional[AngularRates] = None,
        angular_acceleration: Optional[AngularAcceleration] = None,
        general_system_state: Optional[GeneralSystemState] = None
    ):
        self._position = position
        self._velocity = velocity
        self._acceleration = acceleration
        self._orientation = orientation
        self._angular_rates = angular_rates
        self._angular_acceleration = angular_acceleration
        self._general_system_state = general_system_state

    @property
    def position(self) -> Optional[Position]:
        return self._position

    @position.setter
    def position(self, value: Optional[Position]) -> None:
        self._position = value