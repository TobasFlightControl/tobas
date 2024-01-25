import rospy
import rospkg
import os.path as osp
from typing import List
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import MainWidget, IntSliderDisplay, add_spacer

from tobas_msgs.msg import Pwm, PwmArray

PKG_NAME = "tobas_motor_test"
SERVO_RAIL_SIZE = 14


class MotorTestGui(MainWidget):
    MAX_ROWS = SERVO_RAIL_SIZE // 2
    BUTTON_HEIGHT = 30

    MIN_PWM = 1000
    MAX_PWM = 2000
    ARM_PWM = 1100

    def __init__(self) -> None:
        super().__init__(PKG_NAME)

        icon_path = osp.join(rospkg.RosPack().get_path(PKG_NAME), "resources/icon.png")
        self.setWindowIcon(QIcon(icon_path))
        self.setWindowTitle("Motor Test")

        rows = QVBoxLayout()
        self.setLayout(rows)

        grid = QGridLayout()
        rows.addLayout(grid)

        self._commanders: List[IntSliderDisplay] = []
        for i in range(SERVO_RAIL_SIZE):
            commander = IntSliderDisplay(
                f"PIN {i + 1}",
                self.MIN_PWM,
                self.MAX_PWM,
                self.MIN_PWM,
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

        self._pwm_pub = rospy.Publisher("command/pwm", PwmArray, queue_size=1)

    @pyqtSlot()
    def _on_value_changed(self) -> None:
        self._publish_current_values()

    @pyqtSlot()
    def _on_minimum_button_clicked(self) -> None:
        self._set_all_values(self.MIN_PWM)
        self._publish_current_values()

    @pyqtSlot()
    def _on_arming_button_clicked(self) -> None:
        self._set_all_values(self.ARM_PWM)
        self._publish_current_values()

    def _publish_current_values(self) -> None:
        pwms = PwmArray()

        for channel, commander in enumerate(self._commanders):
            pwms.pwm.append(Pwm(channel, commander.get_value()))

        self._pwm_pub.publish(pwms)

    def _set_all_values(self, value: int) -> None:
        for commander in self._commanders:
            commander.blockSignals(True)
            commander.set_value(value)
            commander.blockSignals(False)
