import rospy
import os.path as osp
from typing import List
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import MainWidget, FloatSliderDisplay, add_expanding_widget
from dh_rqt_tools.path import get_proj_path

from tobas_msgs.msg import Throttles

SERVO_RAIL_SIZE = 14
MAX_ROWS = SERVO_RAIL_SIZE // 2


class MotorTestGui(MainWidget):
    def __init__(self) -> None:
        super().__init__()

        proj_path = get_proj_path()
        icon_path = osp.join(proj_path, "resources/icon.png")
        self.setWindowIcon(QIcon(icon_path))
        self.setWindowTitle("Motor Test")

        rows = QVBoxLayout()
        self.setLayout(rows)

        grid = QGridLayout()
        rows.addLayout(grid)

        self._commanders: List[FloatSliderDisplay] = []
        for i in range(SERVO_RAIL_SIZE):
            commander = FloatSliderDisplay(f"PIN {i + 1}", 0.0, 1.0)
            commander.set_value(0.0)
            commander.value_changed.connect(self._on_value_changed)
            self._commanders.append(commander)
            grid.addWidget(commander, i % MAX_ROWS, i // MAX_ROWS)

        add_expanding_widget(rows)

        self._throttles_pub = rospy.Publisher(
            "command/throttles", Throttles, queue_size=1
        )

    @pyqtSlot()
    def _on_value_changed(self) -> None:
        throttles = Throttles()
        throttles.header.stamp = rospy.Time.now()

        for commander in self._commanders:
            throttles.data.append(commander.get_value())

        self._throttles_pub.publish(throttles)
