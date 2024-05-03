from dataclasses import dataclass

from tobas_std_tools_py.enum import ExtEnum


class Commands(ExtEnum):
    WAYPOINT = "Waypoint"
    TAKEOFF = "Takeoff"
    LAND = "Land"
    RETURN_TO_HOME = "Return to Home"


class AltitudeFrame(ExtEnum):
    # MEAN_SEA_LEVEL = "Mean Sea Level"  # TODO: 海面に対する高度指令に対応
    RELATIVE_TO_HOME = "Relative to Home"


@dataclass
class WaypointProperty:
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    altitude_frame: str = ""
    acceptance_radius: float = 0.0
    duration: float = 0.0


@dataclass
class TakeoffProperty:
    altitude: float = 0.0
    altitude_frame: AltitudeFrame = AltitudeFrame.RELATIVE_TO_HOME
    duration: float = 0.0


@dataclass
class LandProperty:
    duration: float = 0.0


@dataclass
class RTHProperty:
    altitude: float = 0.0
    altitude_frame: AltitudeFrame = AltitudeFrame.RELATIVE_TO_HOME
    acceptance_radius: float = 0.0
    duration: float = 0.0
