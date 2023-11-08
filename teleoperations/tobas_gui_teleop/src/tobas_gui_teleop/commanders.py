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
from tobas_msgs.msg import (
    PositionYaw,
    PosVelAccYaw,
    PoseTwistAccelCommand,
    CommandLevel,
    Odometry,
)

from .utils import remap


class CommandersWidget(QScrollArea):
    LABEL_PSIZE = 12

    DEFAULT_INIT_ELEVATION = 0.0  # [m]
    DEFAULT_MIN_X = -10.0  # [m]
    DEFAULT_MAX_X = 10.0  # [m]
    DEFAULT_MIN_Y = -10.0  # [m]
    DEFAULT_MAX_Y = 10.0  # [m]
    DEFAULT_MIN_Z = -3.0  # [m]
    DEFAULT_MAX_Z = 10.0  # [m]
    DEFAULT_MIN_ROLL = -math.pi / 3  # [rad]
    DEFAULT_MAX_ROLL = math.pi / 3  # [rad]
    DEFAULT_MIN_PITCH = -math.pi / 3  # [rad]
    DEFAULT_MAX_PITCH = math.pi / 3  # [rad]
    DEFAULT_MIN_YAW = -math.pi  # [rad]
    DEFAULT_MAX_YAW = math.pi  # [rad]

    def __init__(self, main: GuiTeleopWidget) -> None:
        super().__init__()
        self._main = main

        # RosParams
        self._x_min = 0.0
        self._x_max = 0.0
        self._y_min = 0.0
        self._y_max = 0.0
        self._z_min = 0.0
        self._z_max = 0.0
        self._roll_min = 0.0
        self._roll_max = 0.0
        self._pitch_min = 0.0
        self._pitch_max = 0.0
        self._yaw_min = 0.0
        self._yaw_max = 0.0
        self._init_elevation = 0.0
        self._joint_names: List[str] = []
        self._get_params()

        self.setWidgetResizable(True)  # この設定が必須．無いとオブジェクトが潰れてしまう．

        # QScrollAreaを使う際は，QLayoutの前にQWidgetを挟む必要がある．
        inner = QWidget()
        self.setWidget(inner)
        self._rows = QVBoxLayout()
        inner.setLayout(self._rows)

        self._odom_received = False

        # ドローンの位置姿勢
        drone_label = QLabel("Multirotor Command")
        drone_label.setFont(QFont("Default", self.LABEL_PSIZE, QFont.Bold))
        drone_label.setAlignment(Qt.AlignCenter)
        self._rows.addWidget(drone_label)

        # XYZRPYに対応するバーを追加
        self._drone_cmd_x = Commander("x", self._x_min, self._x_max)
        self._drone_cmd_y = Commander("y", self._y_min, self._y_max)
        self._drone_cmd_z = Commander("z", self._z_min, self._z_max)
        self._drone_cmd_roll = Commander("roll", self._roll_min, self._roll_max)
        self._drone_cmd_pitch = Commander("pitch", self._pitch_min, self._pitch_max)
        self._drone_cmd_yaw = Commander("yaw", self._yaw_min, self._yaw_max)
        self._rows.addWidget(self._drone_cmd_x)
        self._rows.addWidget(self._drone_cmd_y)
        self._rows.addWidget(self._drone_cmd_z)
        self._rows.addWidget(self._drone_cmd_roll)
        self._rows.addWidget(self._drone_cmd_pitch)
        self._rows.addWidget(self._drone_cmd_yaw)

        # 最初はバーを無効化
        self._drone_cmd_x.setEnabled(False)
        self._drone_cmd_y.setEnabled(False)
        self._drone_cmd_z.setEnabled(False)
        self._drone_cmd_roll.setEnabled(False)
        self._drone_cmd_pitch.setEnabled(False)
        self._drone_cmd_yaw.setEnabled(False)

        # その他の可動関節
        robot: Robot = Robot.from_parameter_server("robot_description")
        self.joint_cmds: List[Commander] = []

        if len(self._joint_names) > 0:
            joint_label = QLabel("Joint Command")
            joint_label.setFont(QFont("Default", self.LABEL_PSIZE, QFont.Bold))
            joint_label.setAlignment(Qt.AlignCenter)
            self._rows.addWidget(joint_label)

        for joint_name in self._joint_names:
            joint: Joint = robot.joint_map[joint_name]
            commander = Commander(
                joint_name,
                joint.limit.lower,
                joint.limit.upper,
                f"{joint_name}_controller/command",
            )
            commander.update()
            self.joint_cmds.append(commander)
            self._rows.addWidget(commander)

        # PubSub
        self._pos_yaw_pub = rospy.Publisher(
            "command/position_yaw", PositionYaw, queue_size=1
        )
        self._pvay_pub = rospy.Publisher(
            "command/pos_vel_acc_yaw", PosVelAccYaw, queue_size=1
        )
        self._pta_pub = rospy.Publisher(
            "command/pose_twist_accel", PoseTwistAccelCommand, queue_size=1
        )
        self._odom_sub = rospy.Subscriber("odom", Odometry, self._odom_cb, queue_size=1)

        add_expanding_widget(self._rows)

    def define_connections(self) -> None:
        self._drone_cmd_x.value_changed.connect(self._publish_drone_cmd)
        self._drone_cmd_y.value_changed.connect(self._publish_drone_cmd)
        self._drone_cmd_z.value_changed.connect(self._publish_drone_cmd)
        self._drone_cmd_roll.value_changed.connect(self._publish_drone_cmd)
        self._drone_cmd_pitch.value_changed.connect(self._publish_drone_cmd)
        self._drone_cmd_yaw.value_changed.connect(self._publish_drone_cmd)

    def publish(self) -> None:
        """現在設定されている値を全て発行する．"""
        self._publish_drone_cmd()

        for joint_cmd in self.joint_cmds:
            joint_cmd.publish()

    def _get_params(self) -> None:
        self._x_min = rospy.get_param("~pose_limit/x/min", self.DEFAULT_MIN_X)
        self._x_max = rospy.get_param("~pose_limit/x/max", self.DEFAULT_MAX_X)
        self._y_min = rospy.get_param("~pose_limit/y/min", self.DEFAULT_MIN_Y)
        self._y_max = rospy.get_param("~pose_limit/y/max", self.DEFAULT_MAX_Y)
        self._z_min = rospy.get_param("~pose_limit/z/min", self.DEFAULT_MIN_Z)
        self._z_max = rospy.get_param("~pose_limit/z/max", self.DEFAULT_MAX_Z)
        self._roll_min = rospy.get_param("~pose_limit/roll/min", self.DEFAULT_MIN_ROLL)
        self._roll_max = rospy.get_param("~pose_limit/roll/max", self.DEFAULT_MAX_ROLL)
        self._pitch_min = rospy.get_param(
            "~pose_limit/pitch/min", self.DEFAULT_MIN_PITCH
        )
        self._pitch_max = rospy.get_param(
            "~pose_limit/pitch/max", self.DEFAULT_MAX_PITCH
        )
        self._yaw_min = rospy.get_param("~pose_limit/yaw/min", self.DEFAULT_MIN_YAW)
        self._yaw_max = rospy.get_param("~pose_limit/yaw/max", self.DEFAULT_MAX_YAW)
        self._init_elevation = rospy.get_param(
            "~initial_elevation", self.DEFAULT_INIT_ELEVATION
        )
        self._joint_names = rospy.get_param("posture_defining_joint_names")

        assert self._x_min <= self._x_max
        assert self._y_min <= self._y_max
        assert self._z_min <= self._z_max
        assert self._roll_min <= 0.0 <= self._roll_max
        assert self._pitch_min <= 0.0 <= self._pitch_max
        assert self._yaw_min <= self._yaw_max
        assert self._init_elevation >= 0.0

    @pyqtSlot()
    def _publish_drone_cmd(self) -> None:
        pos_yaw = PositionYaw()
        pos_yaw.level.data = CommandLevel.NORMAL
        pos_yaw.pos.x = self._drone_cmd_x.get_value()
        pos_yaw.pos.y = self._drone_cmd_y.get_value()
        pos_yaw.pos.z = self._drone_cmd_z.get_value()
        pos_yaw.yaw = self._drone_cmd_yaw.get_value()
        self._pos_yaw_pub.publish(pos_yaw)

        pvay = PosVelAccYaw()
        pvay.level.data = CommandLevel.NORMAL
        pvay.pos.x = self._drone_cmd_x.get_value()
        pvay.pos.y = self._drone_cmd_y.get_value()
        pvay.pos.z = self._drone_cmd_z.get_value()
        pvay.yaw = self._drone_cmd_yaw.get_value()
        self._pvay_pub.publish(pvay)

        pta = PoseTwistAccelCommand()
        pta.level.data = CommandLevel.NORMAL
        pta.pos.x = self._drone_cmd_x.get_value()
        pta.pos.y = self._drone_cmd_y.get_value()
        pta.pos.z = self._drone_cmd_z.get_value()
        pta.rpy.roll = self._drone_cmd_roll.get_value()
        pta.rpy.pitch = self._drone_cmd_pitch.get_value()
        pta.rpy.yaw = self._drone_cmd_yaw.get_value()
        self._pta_pub.publish(pta)

    def _odom_cb(self, odom: Odometry) -> None:
        if self._odom_received:
            return

        # 初期コマンドを設定
        self._drone_cmd_x.set_value(odom.pose.pos.x)
        self._drone_cmd_y.set_value(odom.pose.pos.y)
        self._drone_cmd_z.set_value(odom.pose.pos.z + self._init_elevation)
        self._drone_cmd_roll.set_value(0.0)
        self._drone_cmd_pitch.set_value(0.0)
        self._drone_cmd_yaw.set_value(odom.pose.euler.yaw)

        # バーを有効化
        self._drone_cmd_x.setEnabled(True)
        self._drone_cmd_y.setEnabled(True)
        self._drone_cmd_z.setEnabled(True)
        self._drone_cmd_roll.setEnabled(True)
        self._drone_cmd_pitch.setEnabled(True)
        self._drone_cmd_yaw.setEnabled(True)

        self._odom_received = True

        rospy.loginfo("GUI teleoperation is ready.")


class Commander(QWidget):
    PSIZE = 9
    RANGE = 10000

    value_changed = pyqtSignal(float)

    def __init__(
        self, name: str, minimum: float, maximum: float, topic: str = None
    ) -> None:
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
        value = (self._min + self._max) / 2.0
        self.set_value(value)

    def publish(self) -> None:
        if self._topic:
            msg = Float64(data=self.get_value())
            self._value_pub.publish(msg)

    @pyqtSlot()
    def _on_value_changed(self) -> None:
        value = self._slider_to_value()
        self.value.setText(f"{value:.2f}")
        self.value_changed.emit(value)
        self.publish()

    def _slider_to_value(self) -> float:
        x = float(self.slider.value())
        return remap(x, 0.0, self.RANGE, self._min, self._max)

    def _value_to_slider(self, value: float) -> int:
        assert self._min <= value <= self._max
        return int(remap(value, self._min, self._max, 0.0, self.RANGE))
