import os.path as osp
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

SERVO_RAIL_SIZE = 14
MIN_PWM = 1000
MAX_PWM = 2000

PKG_NAME = "tobas_gcs"
TITLE = "Tobas"
UNKNOWN = "Unknown"
SOURCE_CMD = "source /opt/ros/noetic/setup.bash && source /opt/tobas/setup.bash"

# Point Sizes
TITLE_PSIZE = 18
LABEL_PSIZE = 12
BODY_PSIZE = 9

# Raspberry Pi
CATKIN_WS_TOBAS = "/home/pi/.catkin_ws_tobas/"  # Tobasパッケージ用ワークスペース


class Signals(QObject):
    config_pkg_updated = pyqtSignal(str)


class Description(QLabel):
    def __init__(self, text: str) -> None:
        super().__init__(text)

        self.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self.setAlignment(Qt.AlignTop)
        self.setWordWrap(True)
        self.setOpenExternalLinks(True)
