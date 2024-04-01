from dataclasses import dataclass


@dataclass
class BatteryConfig:
    nominal_voltage: float = 0.0
    max_voltage: float = 0.0
    sag_voltage: float = 0.0
    max_current: float = 0.0
