import rospy
import rospkg
import os.path as osp
from typing import List
from functools import partial
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_tools_py.math import rps2rpm, rpm2rps
from tobas_tools_py.drone import Drone
from tobas_rqt_tools.widgets import MainWidget, IntSliderDisplay, add_spacer
from tobas_msgs.msg import RotorSpeeds

from .common import *


class RotorSpeedsPublisherWidget(MainWidget):
    def __init__(self) -> None:
        super().__init__(PKG_NAME)

        drone = Drone()
        drone.load_from_param()

        icon_path = osp.join(rospkg.RosPack().get_path(PKG_NAME), "resources/icon.png")
        self.setWindowIcon(QIcon(icon_path))
        self.setWindowTitle("Motor Test")

        rows = QVBoxLayout()
        self.setLayout(rows)

        self._commanders: List[IntSliderDisplay] = []
        for rotor in drone.rotors:
            commander = IntSliderDisplay(f"CH{rotor.channel}", 0, rps2rpm(rotor.max_rot_speed), 0, suffix=" rpm")
            commander.value_changed.connect(self._on_value_changed)
            rows.addWidget(commander)
            self._commanders.append(commander)

        for rpm in [0, 100, 500, 1000, 5000, 10000]:
            button = QPushButton(f"{rpm} RPM")
            button.setFixedHeight(BUTTON_HEIGHT)
            button.clicked.connect(partial(self._on_rpm_button_clicked, rpm=rpm))
            rows.addWidget(button)

        add_spacer(rows)

        self._speeds_pub = rospy.Publisher("command/rotor_speeds", RotorSpeeds, queue_size=1)

        # モータが停止しないよう一定周期でコマンドを発行し続ける
        rospy.Timer(rospy.Duration(COMMAND_PERIOD), self._command_timer_cb)

    @pyqtSlot()
    def _on_value_changed(self) -> None:
        self._publish_current_values()

    @pyqtSlot()
    def _on_rpm_button_clicked(self, rpm: int) -> None:
        self._set_all_values(rpm)
        self._publish_current_values()

    def _publish_current_values(self) -> None:
        rot_speeds = RotorSpeeds()
        rot_speeds.header.stamp = rospy.Time.now()
        rot_speeds.speeds = [0.0] * len(self._commanders)

        for channel, commander in enumerate(self._commanders):
            rot_speeds.speeds[channel] = rpm2rps(commander.get_value())

        self._speeds_pub.publish(rot_speeds)

    def _set_all_values(self, value: int) -> None:
        for commander in self._commanders:
            commander.blockSignals(True)
            commander.set_value(value)
            commander.blockSignals(False)

    def _command_timer_cb(self, _) -> None:
        self._publish_current_values()
