from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .gui_teleop import GuiTeleopWidget

import math
import random
import rospy
from typing import List
from urdf_parser_py.urdf import Robot, Joint
from std_msgs.msg import Float64
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import Slider, add_expanding_widget
from tobas_msgs.msg import PositionYaw, CommandLevel

from .utils import remap


class CommandersWidget(QScrollArea):

    LABEL_PSIZE = 12
    CONTROL_RATE = 30.  # [Hz]

    INITIAL_X = 0.  # [m]
    INITIAL_Y = 0.  # [m]
    INITIAL_Z = 1.  # [m]
    INITIAL_YAW = 0.  # [rad]

    DEFAULT_MINIMUM_X = -10.  # [m]
    DEFAULT_MAXIMUM_X = 10.  # [m]
    DEFAULT_MINIMUM_Y = -10.  # [m]
    DEFAULT_MAXIMUM_Y = 10.  # [m]
    DEFAULT_MINIMUM_Z = -10.  # [m]
    DEFAULT_MAXIMUM_Z = 10.  # [m]
    DEFAULT_MINIMUM_YAW = -math.pi  # [rad]
    DEFAULT_MAXIMUM_YAW = math.pi  # [rad]

    def __init__(self, main: GuiTeleopWidget) -> None:
        super().__init__()
        self._main = main

        self.setWidgetResizable(True)  # この設定が必須．無いとオブジェクトが潰れてしまう．

        # QScrollAreaを使う際は，QLayoutの前にQWidgetを挟む必要がある．
        inner = QWidget()
        self.setWidget(inner)
        self._rows = QVBoxLayout()
        inner.setLayout(self._rows)

        self._drone_cmd = PositionYaw()
        self._drone_cmd.level.data = CommandLevel.NORMAL

        # ドローンの位置姿勢
        drone_label = QLabel("Multirotor Command")
        drone_label.setFont(QFont("Default", self.LABEL_PSIZE, QFont.Bold))
        drone_label.setAlignment(Qt.AlignCenter)
        self._rows.addWidget(drone_label)

        x_min = rospy.get_param("~pose_limit/x/min", self.DEFAULT_MINIMUM_X)
        x_max = rospy.get_param("~pose_limit/x/max", self.DEFAULT_MAXIMUM_X)
        assert x_min <= x_max
        self.drone_cmd_x = Commander("multirotor/x", x_min, x_max)
        self.drone_cmd_x.set_value(self.INITIAL_X)
        self._rows.addWidget(self.drone_cmd_x)

        y_min = rospy.get_param("~pose_limit/y/min", self.DEFAULT_MINIMUM_Y)
        y_max = rospy.get_param("~pose_limit/y/max", self.DEFAULT_MAXIMUM_Y)
        assert y_min <= y_max
        self.drone_cmd_y = Commander("multirotor/y", y_min, y_max)
        self.drone_cmd_y.set_value(self.INITIAL_Y)
        self._rows.addWidget(self.drone_cmd_y)

        z_min = rospy.get_param("~pose_limit/z/min", self.DEFAULT_MINIMUM_Z)
        z_max = rospy.get_param("~pose_limit/z/max", self.DEFAULT_MAXIMUM_Z)
        assert z_min <= z_max
        self.drone_cmd_z = Commander("multirotor/z", z_min, z_max)
        self.drone_cmd_z.set_value(self.INITIAL_Z)
        self._rows.addWidget(self.drone_cmd_z)

        yaw_min = rospy.get_param("~pose_limit/yaw/min", self.DEFAULT_MINIMUM_YAW)
        yaw_max = rospy.get_param("~pose_limit/yaw/max", self.DEFAULT_MAXIMUM_YAW)
        assert yaw_min <= yaw_max
        self.drone_cmd_yaw = Commander("multirotor/yaw", yaw_min, yaw_max)
        self.drone_cmd_yaw.set_value(self.INITIAL_YAW)
        self._rows.addWidget(self.drone_cmd_yaw)

        # その他の可動関節
        joint_names = rospy.get_param("active_joint_names")
        robot: Robot = Robot.from_parameter_server("robot_description")
        self.joint_cmds: List[Commander] = []

        if len(joint_names) > 0:
            joint_label = QLabel("Joint Command")
            joint_label.setFont(QFont("Default", self.LABEL_PSIZE, QFont.Bold))
            joint_label.setAlignment(Qt.AlignCenter)
            self._rows.addWidget(joint_label)

        for joint_name in joint_names:
            joint: Joint = robot.joint_map[joint_name]
            commander = Commander(
                joint_name,
                joint.limit.lower,
                joint.limit.upper,
                f'{joint_name}_controller/command',
            )
            commander.update()
            self.joint_cmds.append(commander)
            self._rows.addWidget(commander)

        # Publisher
        self._drone_cmd_pub = rospy.Publisher("command/position_yaw", PositionYaw, queue_size=1)

        add_expanding_widget(self._rows)

    def define_connections(self) -> None:
        self.drone_cmd_x.value_changed.connect(self._publish_drone_cmd)
        self.drone_cmd_y.value_changed.connect(self._publish_drone_cmd)
        self.drone_cmd_z.value_changed.connect(self._publish_drone_cmd)
        self.drone_cmd_yaw.value_changed.connect(self._publish_drone_cmd)

    def publish(self) -> None:
        """ 現在設定されている値を全て発行する． """
        self._publish_drone_cmd()

        for joint_cmd in self.joint_cmds:
            joint_cmd.publish()

    @pyqtSlot()
    def _publish_drone_cmd(self) -> None:
        self._drone_cmd.pos.x = self.drone_cmd_x.get_value()
        self._drone_cmd.pos.y = self.drone_cmd_y.get_value()
        self._drone_cmd.pos.z = self.drone_cmd_z.get_value()
        self._drone_cmd.yaw = self.drone_cmd_yaw.get_value()

        self._drone_cmd_pub.publish(self._drone_cmd)


class Commander(QWidget):

    PSIZE = 9
    RANGE = 10000

    value_changed = pyqtSignal(float)

    def __init__(self, name: str, minimum: float, maximum: float, topic: str = None) -> None:
        super().__init__()
        self._min = minimum
        self._max = maximum
        self._topic = topic

        font = QFont("Default", self.PSIZE, QFont.Bold)

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        self._cols = QHBoxLayout()
        self._rows.addLayout(self._cols)

        self.name = QLabel(name)
        self.name.setFont(font)
        self._cols.addWidget(self.name)

        self.value = QLineEdit("0.00")
        self.value.setAlignment(Qt.AlignRight)
        self.value.setFont(font)
        self.value.setReadOnly(True)
        self.value.setFocusPolicy(Qt.NoFocus)
        self._cols.addWidget(self.value)

        self.slider = Slider(Qt.Horizontal)
        self.slider.setFont(font)
        self.slider.setRange(0, self.RANGE)
        self.slider.setValue(self.RANGE // 2)
        self._rows.addWidget(self.slider)

        if self._topic:
            self._value_pub = rospy.Publisher(topic, Float64, queue_size=1)

        self.slider.valueChanged.connect(self._on_value_changed)

    def get_value(self) -> float:
        return float(self.value.text())

    def set_value(self, value: float) -> None:
        slider_value = self._value_to_slider(value)
        self.slider.setValue(slider_value)

    def set_random_value(self) -> None:
        value = random.uniform(self._min, self._max)
        self.set_value(value)

    def set_center_value(self) -> None:
        value = (self._min + self._max) / 2.
        self.set_value(value)

    def publish(self) -> None:
        if self._topic:
            msg = Float64(data=self.get_value())
            self._value_pub.publish(msg)

    @pyqtSlot()
    def _on_value_changed(self) -> None:
        value = self._slider_to_value()
        self.value.setText(f'{value:.2f}')
        self.value_changed.emit(value)
        self.publish()

    def _slider_to_value(self) -> float:
        x = float(self.slider.value())
        return remap(x, 0., self.RANGE, self._min, self._max)

    def _value_to_slider(self, value: float) -> int:
        assert self._min <= value <= self._max
        return int(remap(value, self._min, self._max, 0., self.RANGE))
