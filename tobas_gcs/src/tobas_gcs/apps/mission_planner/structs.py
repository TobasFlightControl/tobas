from tobas_std_tools_py.enum import ExtEnum


class Commands(ExtEnum):
    WAYPOINT = "Waypoint"
    TAKEOFF = "Takeoff"
    LAND = "Land"
    RETURN_TO_HOME = "Return to Home"


class AltitudeFrame(ExtEnum):
    # MEAN_SEA_LEVEL = "Mean Sea Level"  # TODO: 海面に対する高度指令に対応
    RELATIVE_TO_HOME = "Relative to Home"
