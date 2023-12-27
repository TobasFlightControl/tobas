import os.path as osp
import math
import rospy
import rospkg
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import MainWidget, FloatSliderDisplay, add_spacer
from tobas_msgs.msg import (
    PositionYaw,
    PosVelAccYaw,
    PoseTwistAccelCommand,
    CommandLevel,
    Odometry,
)

from .common import *


class BasePoseCommander(MainWidget):
    # Constants
    HOME_ALTITUDE = 3.0  # [ m]

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

    def __init__(self) -> None:
        super().__init__(f"{PKG_NAME}/base_pose_commander")

        icon_path = osp.join(rospkg.RosPack().get_path(PKG_NAME), "resources/icon.png")
        self.setWindowIcon(QIcon(icon_path))
        self.setWindowTitle("Base State Commander")

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

        # XYZRPYに対応するバーを追加
        self._cmd_x = FloatSliderDisplay(
            "x",
            self._x_min,
            self._x_max,
            0.0,
            callback=self._publish_current_command,
        )
        self._cmd_y = FloatSliderDisplay(
            "y",
            self._y_min,
            self._y_max,
            0.0,
            callback=self._publish_current_command,
        )
        self._cmd_z = FloatSliderDisplay(
            "z",
            self._z_min,
            self._z_max,
            0.0,
            callback=self._publish_current_command,
        )
        self._cmd_roll = FloatSliderDisplay(
            "roll",
            self._roll_min,
            self._roll_max,
            0.0,
            callback=self._publish_current_command,
        )
        self._cmd_pitch = FloatSliderDisplay(
            "pitch",
            self._pitch_min,
            self._pitch_max,
            0.0,
            callback=self._publish_current_command,
        )
        self._cmd_yaw = FloatSliderDisplay(
            "yaw",
            self._yaw_min,
            self._yaw_max,
            0.0,
            callback=self._publish_current_command,
        )
        rows.addWidget(self._cmd_x)
        rows.addWidget(self._cmd_y)
        rows.addWidget(self._cmd_z)
        rows.addWidget(self._cmd_roll)
        rows.addWidget(self._cmd_pitch)
        rows.addWidget(self._cmd_yaw)

        # 初期位置にボタン
        self._home_button = QPushButton("Home", parent=self)
        self._home_button.setFixedHeight(BUTTON_HEIGHT)
        self._home_button.clicked.connect(self._on_home_button_clicked)
        rows.addWidget(self._home_button)

        # スペーサー
        add_spacer(rows)

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
