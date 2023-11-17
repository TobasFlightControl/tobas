from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .gui_teleop import GuiTeleopWidget

import math
import random
import rospy
from typing import List, Callable
from urdf_parser_py.urdf import Robot, Joint, JointLimit
from std_msgs.msg import Float64
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.utils import remap
from dh_rqt_tools.widgets import Slider, add_expanding_widget
from tobas_msgs.msg import (
    PositionYaw,
    PosVelAccYaw,
    PoseTwistAccelCommand,
    CommandLevel,
    Odometry,
)

from .common import *


class MultirotorCommanderWidget(QWidget):
    # Constants
    HOME_ALTITUDE = 3.0

    # Default parameters
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

        # rosparams
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
        self._get_params()

        # メインレイアウト
        rows = QVBoxLayout()
        self.setLayout(rows)

        # ラベル
        label = QLabel("Multirotor Command")
        label.setFont(QFont("Default", LABEL_PSIZE, QFont.Bold))
        label.setAlignment(Qt.AlignCenter)
        rows.addWidget(label)

        # XYZRPYに対応するバーを追加
        self._cmd_x = Commander(
            "x",
            self._x_min,
            self._x_max,
            callback=self._publish_current_command,
        )
        self._cmd_y = Commander(
            "y",
            self._y_min,
            self._y_max,
            callback=self._publish_current_command,
        )
        self._cmd_z = Commander(
            "z",
            self._z_min,
            self._z_max,
            callback=self._publish_current_command,
        )
        self._cmd_roll = Commander(
            "roll",
            self._roll_min,
            self._roll_max,
            callback=self._publish_current_command,
        )
        self._cmd_pitch = Commander(
            "pitch",
            self._pitch_min,
            self._pitch_max,
            callback=self._publish_current_command,
        )
        self._cmd_yaw = Commander(
            "yaw",
            self._yaw_min,
            self._yaw_max,
            callback=self._publish_current_command,
        )
        rows.addWidget(self._cmd_x)
        rows.addWidget(self._cmd_y)
        rows.addWidget(self._cmd_z)
        rows.addWidget(self._cmd_roll)
        rows.addWidget(self._cmd_pitch)
        rows.addWidget(self._cmd_yaw)

        # 初期位置にボタン
        self._home_button = QPushButton("Home")
        self._home_button.clicked.connect(self._on_home_button_clicked)
        rows.addWidget(self._home_button)

        # 最初はバーを無効化
        self._cmd_x.setEnabled(False)
        self._cmd_y.setEnabled(False)
        self._cmd_z.setEnabled(False)
        self._cmd_roll.setEnabled(False)
        self._cmd_pitch.setEnabled(False)
        self._cmd_yaw.setEnabled(False)

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

    def publish_current_command(self) -> None:
        """現在設定されている値を全て発行する．"""
        self._publish_current_command()

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

        assert self._x_min <= 0.0 <= self._x_max
        assert self._y_min <= 0.0 <= self._y_max
        assert self._z_min <= self._z_max
        assert self._roll_min <= 0.0 <= self._roll_max
        assert self._pitch_min <= 0.0 <= self._pitch_max
        assert self._yaw_min <= 0.0 <= self._yaw_max
        assert self._init_elevation >= 0.0

    def _odom_cb(self, odom: Odometry) -> None:
        # 初期コマンドを設定
        self._cmd_x.set_value(odom.pose.pos.x)
        self._cmd_y.set_value(odom.pose.pos.y)
        self._cmd_z.set_value(odom.pose.pos.z + self._init_elevation)
        self._cmd_roll.set_value(0.0)
        self._cmd_pitch.set_value(0.0)
        self._cmd_yaw.set_value(odom.pose.euler.yaw)

        # バーを有効化
        self._cmd_x.setEnabled(True)
        self._cmd_y.setEnabled(True)
        self._cmd_z.setEnabled(True)
        self._cmd_roll.setEnabled(True)
        self._cmd_pitch.setEnabled(True)
        self._cmd_yaw.setEnabled(True)

        rospy.loginfo("GUI teleoperation is ready.")
        self._odom_sub.unregister()

    @pyqtSlot()
    def _publish_current_command(self) -> None:
        pos_yaw = PositionYaw()
        pos_yaw.level.data = CommandLevel.NORMAL
        pos_yaw.pos.x = self._cmd_x.get_value()
        pos_yaw.pos.y = self._cmd_y.get_value()
        pos_yaw.pos.z = self._cmd_z.get_value()
        pos_yaw.yaw = self._cmd_yaw.get_value()
        self._pos_yaw_pub.publish(pos_yaw)

        pvay = PosVelAccYaw()
        pvay.level.data = CommandLevel.NORMAL
        pvay.pos.x = self._cmd_x.get_value()
        pvay.pos.y = self._cmd_y.get_value()
        pvay.pos.z = self._cmd_z.get_value()
        pvay.yaw = self._cmd_yaw.get_value()
        self._pvay_pub.publish(pvay)

        pta = PoseTwistAccelCommand()
        pta.level.data = CommandLevel.NORMAL
        pta.pos.x = self._cmd_x.get_value()
        pta.pos.y = self._cmd_y.get_value()
        pta.pos.z = self._cmd_z.get_value()
        pta.rpy.roll = self._cmd_roll.get_value()
        pta.rpy.pitch = self._cmd_pitch.get_value()
        pta.rpy.yaw = self._cmd_yaw.get_value()
        self._pta_pub.publish(pta)

    @pyqtSlot()
    def _on_home_button_clicked(self) -> None:
        self._cmd_x.set_value(0.0)
        self._cmd_y.set_value(0.0)
        self._cmd_z.set_value(self.HOME_ALTITUDE)
        self._cmd_roll.set_value(0.0)
        self._cmd_pitch.set_value(0.0)
        self._cmd_yaw.set_value(0.0)


class JointPositionCommanderWidget(QWidget):
    INTERVAL = 0.1

    def __init__(self, main: GuiTeleopWidget) -> None:
        super().__init__()
        self._main = main

        # Commanders
        self._commanders: List[Commander] = []

        # rosparams
        self._joint_names: List[str] = []
        self._get_params()

        # 可動関節が無い場合は終了
        if len(self._joint_names) == 0:
            return

        # メインレイアウト
        rows = QVBoxLayout()
        self.setLayout(rows)

        # ラベル
        label = QLabel("Joint Position Command")
        label.setFont(QFont("Default", LABEL_PSIZE, QFont.Bold))
        label.setAlignment(Qt.AlignCenter)
        rows.addWidget(label)

        # Commandersをセット
        robot: Robot = Robot.from_parameter_server("robot_description")
        for joint_name in self._joint_names:
            joint: Joint = robot.joint_map[joint_name]
            limit: JointLimit = joint.limit
            commander = Commander(
                joint_name,
                limit.lower,
                limit.upper,
                f"{joint_name}_controller/command",
            )
            commander.update()
            self._commanders.append(commander)
            rows.addWidget(commander)

        self._random_button = QPushButton("Randomize")
        self._random_button.setFixedHeight(BUTTON_HEIGHT)
        self._random_button.clicked.connect(self._on_random_button_clicked)
        rows.addWidget(self._random_button)

        self._center_button = QPushButton("Center")
        self._center_button.setFixedHeight(BUTTON_HEIGHT)
        self._center_button.clicked.connect(self._on_center_button_clicked)
        rows.addWidget(self._center_button)

        add_expanding_widget(rows)

    def publish_current_command(self) -> None:
        """現在設定されている値を全て発行する．"""
        for joint_cmd in self._commanders:
            joint_cmd.publish_current_command()

    def _get_params(self) -> None:
        self._joint_names = rospy.get_param("posture_defining_joint_names")

    @pyqtSlot()
    def _on_random_button_clicked(self) -> None:
        """全ての関節角をランダム値に設定する．"""
        self.setEnabled(False)

        # 一気に指令すると反映されないので，間隔を開けながら指令する
        for joint_cmd in self._commanders:
            joint_cmd.set_random_value()
            rospy.sleep(self.INTERVAL)

        self.setEnabled(True)

    @pyqtSlot()
    def _on_center_button_clicked(self) -> None:
        """全ての関節角を中央の値に設定する．"""
        self.setEnabled(False)

        for joint_cmd in self._commanders:
            joint_cmd.set_center_value()
            rospy.sleep(self.INTERVAL)

        self.setEnabled(True)


class Commander(QWidget):
    PSIZE = 9
    RANGE = 10000

    value_changed = pyqtSignal(float)

    def __init__(
        self,
        name: str,
        minimum: float,
        maximum: float,
        pub_topic: str = None,
        callback: Callable[[float], None] = None,
    ) -> None:
        super().__init__()
        self._min = minimum
        self._max = maximum
        self._pub_topic = pub_topic
        self._callback = callback

        font = QFont("Default", self.PSIZE, QFont.Bold)

        rows = QVBoxLayout()
        self.setLayout(rows)

        cols = QHBoxLayout()
        rows.addLayout(cols)

        name = QLabel(name)
        name.setFont(font)
        cols.addWidget(name)

        self._value = QLineEdit("0.00")
        self._value.setAlignment(Qt.AlignRight)
        self._value.setFont(font)
        self._value.setReadOnly(True)
        self._value.setFocusPolicy(Qt.NoFocus)
        cols.addWidget(self._value)

        self._slider = Slider(Qt.Horizontal)
        self._slider.setFont(font)
        self._slider.setRange(0, self.RANGE)
        self._slider.setValue(self.RANGE // 2)
        rows.addWidget(self._slider)

        if self._pub_topic is not None:
            self._value_pub = rospy.Publisher(pub_topic, Float64, queue_size=1)

        if self._callback is not None:
            self.value_changed.connect(self._callback)

        self._slider.valueChanged.connect(self._on_value_changed)

    def get_value(self) -> float:
        return float(self._value.text())

    def set_value(self, value: float) -> None:
        slider_value = self._value_to_slider(value)
        self._slider.setValue(slider_value)

    def set_random_value(self) -> None:
        value = random.uniform(self._min, self._max)
        self.set_value(value)

    def set_center_value(self) -> None:
        value = (self._min + self._max) / 2
        self.set_value(value)

    def publish_current_command(self) -> None:
        if self._pub_topic is None:
            return

        msg = Float64(data=self.get_value())
        self._value_pub.publish(msg)

    @pyqtSlot()
    def _on_value_changed(self) -> None:
        value = self._slider_to_value()
        self._value.setText(f"{value:.2f}")
        self.value_changed.emit(value)
        self.publish_current_command()

    def _slider_to_value(self) -> float:
        x = float(self._slider.value())
        return remap(x, 0.0, self.RANGE, self._min, self._max)

    def _value_to_slider(self, value: float) -> int:
        assert self._min <= value <= self._max
        return int(remap(value, self._min, self._max, 0.0, self.RANGE))
