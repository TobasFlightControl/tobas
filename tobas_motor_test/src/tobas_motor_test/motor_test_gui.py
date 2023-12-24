import rospy
import rospkg
import os.path as osp
from typing import List
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import MainWidget, FloatSliderDisplay, add_spacer

from tobas_msgs.msg import Throttles

PKG_NAME = "tobas_motor_test"
SERVO_RAIL_SIZE = 14


class MotorTestGui(MainWidget):
    MAX_ROWS = SERVO_RAIL_SIZE // 2
    BUTTON_HEIGHT = 30

    MIN_THROTTLE = 0.0
    MAX_THROTTLE = 1.0
    ARM_THROTTLE = 0.1

    def __init__(self) -> None:
        super().__init__(PKG_NAME)

        icon_path = osp.join(rospkg.RosPack().get_path(PKG_NAME), "resources/icon.png")
        self.setWindowIcon(QIcon(icon_path))
        self.setWindowTitle("Motor Test")

        rows = QVBoxLayout()
        self.setLayout(rows)

        grid = QGridLayout()
        rows.addLayout(grid)

        self._commanders: List[FloatSliderDisplay] = []
        for i in range(SERVO_RAIL_SIZE):
            commander = FloatSliderDisplay(
                f"PIN {i + 1}",
                self.MIN_THROTTLE,
                self.MAX_THROTTLE,
                self.MIN_THROTTLE,
            )
            commander.value_changed.connect(self._on_value_changed)
            self._commanders.append(commander)
            grid.addWidget(commander, i % self.MAX_ROWS, i // self.MAX_ROWS)

        self._minimum_button = QPushButton("Minimum")
        self._minimum_button.clicked.connect(self._on_minimum_button_clicked)
        self._minimum_button.setFixedHeight(self.BUTTON_HEIGHT)
        rows.addWidget(self._minimum_button)

        self._arming_button = QPushButton("Arming")
        self._arming_button.clicked.connect(self._on_arming_button_clicked)
        self._arming_button.setFixedHeight(self.BUTTON_HEIGHT)
        rows.addWidget(self._arming_button)

        add_spacer(rows)

        self._throttles_pub = rospy.Publisher(
            "command/throttles", Throttles, queue_size=1
        )

    @pyqtSlot()
    def _on_value_changed(self) -> None:
        self._publish_current_values()

    @pyqtSlot()
    def _on_minimum_button_clicked(self) -> None:
        self._set_all_values(self.MIN_THROTTLE)
        self._publish_current_values()

    @pyqtSlot()
    def _on_arming_button_clicked(self) -> None:
        self._set_all_values(self.ARM_THROTTLE)
        self._publish_current_values()

    def _publish_current_values(self) -> None:
        throttles = Throttles()
        throttles.header.stamp = rospy.Time.now()

        for commander in self._commanders:
            throttles.data.append(commander.get_value())

        self._throttles_pub.publish(throttles)

    def _set_all_values(self, value: float) -> None:
        for commander in self._commanders:
            commander.blockSignals(True)
            commander.set_value(value)
            commander.blockSignals(False)
