import os.path as osp
import math
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


# Physics Constants
AIR_DENSITY = 1.225  # [kg/m^3]

PKG_NAME = "tobas_setup_assistant"
TITLE = "Tobas Setup Assistant"
TO_DO = "TODO"
CW = "CW"
CCW = "CCW"

# Point Sizes
TITLE_PSIZE = 18
LABEL_PSIZE = 12
BODY_PSIZE = 9

# Default Parameters
DEFAULT_NUM_FLIGHT_MODES = 2


ROSLAUNCH_TIMEOUT = 5  # [s]
PROP_TILT_TOL = math.radians(5)  # 軸と平行とみなす傾きの閾値

SENSOR_OFFSET_DESCRIPTION = "The pose of the sensor frame wrt. the the drone root frame."
CAMERA_OFFSET_DESCRIPTION = "The pose of the camera frame wrt. the the selected link frame."


class Signals(QObject):
    urdf_loaded = pyqtSignal()
    airframe_updated = pyqtSignal()


class Description(QLabel):
    def __init__(self, text: str) -> None:
        super().__init__(text)

        self.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self.setAlignment(Qt.AlignTop)
        self.setWordWrap(True)
        self.setOpenExternalLinks(True)
