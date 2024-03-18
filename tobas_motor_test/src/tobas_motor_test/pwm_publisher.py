import rospy
import rospkg
import os.path as osp
from typing import List
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import MainWidget, IntSliderDisplay
from tobas_msgs.msg import Pwm, PwmArray

from .common import *


class PwmPublisherWidget(MainWidget):
    MAX_ROWS = SERVO_RAIL_SIZE // 2

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
        for channel in range(SERVO_RAIL_SIZE):
            commander = IntSliderDisplay(self)
            commander.set_text(f"CH{channel}")
            commander.set_minimum(MIN_PWM)
            commander.set_maximum(MAX_PWM)
            commander.set_value(MIN_PWM)
            commander.set_suffix(" us")
            commander.value_changed.connect(self._on_value_changed)
            self._commanders.append(commander)
            grid.addWidget(commander, channel % self.MAX_ROWS, channel // self.MAX_ROWS)

        self._minimum_button = QPushButton("Minimum")
        self._minimum_button.clicked.connect(self._on_minimum_button_clicked)
        self._minimum_button.setFixedHeight(BUTTON_HEIGHT)
        rows.addWidget(self._minimum_button)

        rows.addStretch()

        self._pwm_pub = rospy.Publisher("command/pwm", PwmArray, queue_size=1)

        # モータが停止しないよう一定周期でコマンドを発行し続ける
        rospy.Timer(rospy.Duration(COMMAND_PERIOD), self._command_timer_cb)

    @pyqtSlot()
    def _on_value_changed(self) -> None:
        self._publish_current_values()

    @pyqtSlot()
    def _on_minimum_button_clicked(self) -> None:
        self._set_all_values(MIN_PWM)
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

    def _command_timer_cb(self, _) -> None:
        self._publish_current_values()
