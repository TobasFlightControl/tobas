from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import rospy
from typing import List
from functools import partial
from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_tools_py.math import rps2rpm, rpm2rps
from tobas_rqt_tools.messages import q_error
from tobas_rqt_tools.widgets import IntSliderDisplay
from tobas_tools_py.drone import Drone
from tobas_msgs.msg import RotorSpeeds
from tobas_msgs.srv import GetArm, GetArmRequest, GetArmResponse, SetArm, SetArmRequest, SetArmResponse

from ....common import *
from .base import BaseHardwareSetupWidget


class MotorTestWidget(BaseHardwareSetupWidget):
    NAME = "Motor Test"
    TITLE = "Test Motors"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        warning = Description("Warning: Ensure that propellers are removed from motors.\n\n")
        warning.setStyleSheet("color: red; font-weight: bold;")
        self._rows.addWidget(warning)

        instruction = Description(
            "1. Connect the ESCs to the Navio2 in the correct order.\n\n"
            '2. Press "Start" button.\n\n'
            "3. For all motors, confirm the followings:\n"
            "   - The motor rotates in the correct direction. If not, swap any two of the three ESC-motor connections.\n"
            "   - The motor does not rotate when the command RPM is 0.\n"
            "   - The sound of rotation gradually increases as the command RPM approaches its maximum value.\n"
            "   - Two motors of the same model produce roughly the same sound level at the same command RPM.\n\n"
            '4. Press "Stop" button.\n\n'
        )
        self._rows.addWidget(instruction)

        cols = QHBoxLayout()
        self._rows.addLayout(cols)

        self._start_button = QPushButton("Start")
        self._start_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        cols.addWidget(self._start_button)

        self._stop_button = QPushButton("Stop")
        self._stop_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._stop_button.setEnabled(False)
        cols.addWidget(self._stop_button)

        cols.addStretch()

        self._rotor_speeds_publisher = RotorSpeedsPublisherWidget(self._drone)
        self._rows.addWidget(self._rotor_speeds_publisher)

        self._rows.addStretch()

    @override
    def define_connections(self) -> None:
        self._start_button.clicked.connect(self._on_start_button_clicked)
        self._stop_button.clicked.connect(self._on_stop_button_clicked)

    @override
    def update_internal_data_structures(self) -> None:
        self._rotor_speeds_publisher.update_internal_data_structures()

    @pyqtSlot()
    def _on_start_button_clicked(self) -> None:
        if not self._check_disarm():
            return

        if not self._set_arm(True):
            return

        self._rotor_speeds_publisher.start()

        self._start_button.setEnabled(False)
        self._stop_button.setEnabled(True)

    @pyqtSlot()
    def _on_stop_button_clicked(self) -> None:
        if not self._set_arm(False):
            return

        self._rotor_speeds_publisher.stop()

        self._start_button.setEnabled(True)
        self._stop_button.setEnabled(False)

    def _check_disarm(self) -> bool:
        get_arm_sc = rospy.ServiceProxy(f"/{self._drone.drone_name}/get_arm", GetArm)
        try:
            get_arm_sc.wait_for_service(self.WAIT_FOR_SERVER)
        except rospy.ROSException:
            q_error(self, self.E_FAILED_TO_CONNECT)
            return False

        try:
            res: GetArmResponse = get_arm_sc.call(GetArmRequest())
        except Exception as e:
            q_error(self, f"{self.E_FAILED_TO_CALL_SRV}: {e}")
            return

        if res.arming:
            q_error(self, "Cannot start motor test because they are already armed.")
            return False

        return True

    def _set_arm(self, arming: bool) -> bool:
        set_arm_sc = rospy.ServiceProxy(f"/{self._drone.drone_name}/set_arm", SetArm)
        try:
            set_arm_sc.wait_for_service(self.WAIT_FOR_SERVER)
        except rospy.ROSException:
            q_error(self, self.E_FAILED_TO_CONNECT)
            return False

        req = SetArmRequest()
        req.arming = arming

        try:
            res: SetArmResponse = set_arm_sc.call(req)
        except Exception as e:
            q_error(self, f"{self.E_FAILED_TO_CALL_SRV}: {e}")
            return

        if not res.success:
            q_error(self, res.message)
            return False

        return True


class RotorSpeedsPublisherWidget(QWidget):

    MAX_ROWS = SERVO_RAIL_SIZE // 2
    BUTTON_HEIGHT = 50
    COMMAND_PERIOD = 0.1  # [s]

    def __init__(self, drone: Drone) -> None:
        super().__init__()
        self._drone = drone

        rows = QVBoxLayout()
        self.setLayout(rows)

        grid = QGridLayout()
        rows.addLayout(grid)

        self._commanders: List[IntSliderDisplay] = []
        for channel in range(SERVO_RAIL_SIZE):
            commander = IntSliderDisplay()
            commander.set_suffix(" rpm")
            commander.value_changed.connect(self._on_value_changed)
            self._commanders.append(commander)
            grid.addWidget(commander, channel % self.MAX_ROWS, channel // self.MAX_ROWS)

        cols = QHBoxLayout()
        rows.addLayout(cols)

        self._rpm_buttons: List[QPushButton] = []
        for rpm in [0, 100, 500, 1000, 5000, 10000]:
            button = QPushButton(f"{rpm} RPM")
            button.setFixedHeight(self.BUTTON_HEIGHT)
            button.clicked.connect(partial(self._on_rpm_button_clicked, rpm=rpm))
            self._rpm_buttons.append(button)
            cols.addWidget(button)

        self._speeds_pub = None
        self._publish_timer = None

        self.update_internal_data_structures()

    def update_internal_data_structures(self) -> None:
        # モータとして登録されているチャンネルの設定
        rotor_channels = set()
        for rotor in self._drone.rotors:
            rotor_channels.add(rotor.channel)
            self._commanders[rotor.channel].set_text(f"CH{rotor.channel}: {rotor.link_name}")
            self._commanders[rotor.channel].set_maximum(rps2rpm(rotor.max_rot_speed))
            self._commanders[rotor.channel].set_value(0)

        # 最初は無効化
        self.stop()

        # モータとして登録されていないチャンネルを無効化
        for channel in range(SERVO_RAIL_SIZE):
            if channel in rotor_channels:
                continue
            self._commanders[channel].set_text(f"CH{channel}: unregistered")
            self._commanders[channel].set_maximum(0)
            self._commanders[channel].setEnabled(False)

        # 回転数トピックを更新
        if self._speeds_pub is not None:
            self._speeds_pub.unregister()
        self._speeds_pub = rospy.Publisher(f"{self._drone.drone_name}/command/rotor_speeds", RotorSpeeds, queue_size=1)

    def start(self) -> None:
        # コマンダーを有効化
        for rotor in self._drone.rotors:
            self._commanders[rotor.channel].set_value(0)
            self._commanders[rotor.channel].setEnabled(True)

        # RPMボタンを有効化
        for button in self._rpm_buttons:
            button.setEnabled(True)

        # モータが停止しないよう一定周期でコマンドを発行し続ける
        self._publish_timer = rospy.Timer(rospy.Duration(self.COMMAND_PERIOD), self._publish_timer_cb, False)

    def stop(self) -> None:
        # コマンダーを無効化
        for rotor in self._drone.rotors:
            self._commanders[rotor.channel].set_value(0)
            self._commanders[rotor.channel].setEnabled(False)

        # RPMボタンを無効化
        for button in self._rpm_buttons:
            button.setEnabled(False)

        # タイマーを停止
        if self._publish_timer is not None:
            self._publish_timer.shutdown()

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
        rot_speeds.speeds = [0.0] * len(self._drone.rotors)

        for rotor in self._drone.rotors:
            rot_speeds.speeds[rotor.channel] = rpm2rps(self._commanders[rotor.channel].get_value())

        self._speeds_pub.publish(rot_speeds)

    def _set_all_values(self, value: int) -> None:
        for rotor in self._drone.rotors:
            self._commanders[rotor.channel].set_value(value)

    def _publish_timer_cb(self, _) -> None:
        self._publish_current_values()
