import math
import rclpy
from rclpy.node import Node
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QPushButton, QVBoxLayout

from tobas_std_tools_py.geometry import euler_from_matrix
from tobas_rqt_tools.widgets import Widget, FloatSliderDisplay
from tobas_msgs.msg import (
    PositionYaw,
    PosVelAccYaw,
    PoseTwistAccelCommand,
    CommandLevel,
    Odometry,
)
from tobas_msgs.srv import SetArm

from .common import BUTTON_HEIGHT


class BasePoseCommanderWidget(Widget):
    # Constants
    HOME_ALTITUDE = 3.0  # [m]
    WAIT_FOR_SERVICE = 1.0  # [s]
    ARM_RETRY_INTERVAL = 1.0  # [s]

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

    def __init__(self, node: Node) -> None:
        super().__init__()
        self._node = node

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
        self._cmd_x = FloatSliderDisplay()
        self._cmd_x.set_text("x")
        self._cmd_x.set_minimum(self._x_min)
        self._cmd_x.set_maximum(self._x_max)
        self._cmd_x.set_value(0.0)
        self._cmd_x.set_callback(self._publish_current_command)
        self._cmd_x.setEnabled(False)
        rows.addWidget(self._cmd_x)

        self._cmd_y = FloatSliderDisplay()
        self._cmd_y.set_text("y")
        self._cmd_y.set_minimum(self._y_min)
        self._cmd_y.set_maximum(self._y_max)
        self._cmd_y.set_value(0.0)
        self._cmd_y.set_callback(self._publish_current_command)
        self._cmd_y.setEnabled(False)
        rows.addWidget(self._cmd_y)

        self._cmd_z = FloatSliderDisplay()
        self._cmd_z.set_text("z")
        self._cmd_z.set_minimum(self._z_min)
        self._cmd_z.set_maximum(self._z_max)
        self._cmd_z.set_value(0.0)
        self._cmd_z.set_callback(self._publish_current_command)
        self._cmd_z.setEnabled(False)
        rows.addWidget(self._cmd_z)

        self._cmd_roll = FloatSliderDisplay()
        self._cmd_roll.set_text("roll")
        self._cmd_roll.set_minimum(self._roll_min)
        self._cmd_roll.set_maximum(self._roll_max)
        self._cmd_roll.set_value(0.0)
        self._cmd_roll.set_callback(self._publish_current_command)
        self._cmd_roll.setEnabled(False)
        rows.addWidget(self._cmd_roll)

        self._cmd_pitch = FloatSliderDisplay()
        self._cmd_pitch.set_text("pitch")
        self._cmd_pitch.set_minimum(self._pitch_min)
        self._cmd_pitch.set_maximum(self._pitch_max)
        self._cmd_pitch.set_value(0.0)
        self._cmd_pitch.set_callback(self._publish_current_command)
        self._cmd_pitch.setEnabled(False)
        rows.addWidget(self._cmd_pitch)

        self._cmd_yaw = FloatSliderDisplay()
        self._cmd_yaw.set_text("yaw")
        self._cmd_yaw.set_minimum(self._yaw_min)
        self._cmd_yaw.set_maximum(self._yaw_max)
        self._cmd_yaw.set_value(0.0)
        self._cmd_yaw.set_callback(self._publish_current_command)
        self._cmd_yaw.setEnabled(False)
        rows.addWidget(self._cmd_yaw)

        # 初期位置にボタン
        self._home_button = QPushButton("Home")
        self._home_button.setFixedHeight(BUTTON_HEIGHT)
        self._home_button.clicked.connect(self._on_home_button_clicked)
        rows.addWidget(self._home_button)

        # スペーサー
        rows.addStretch()

        # PubSub
        self._pos_yaw_pub = self._node.create_publisher(PositionYaw, "command/position_yaw", 1)
        self._pvay_pub = self._node.create_publisher(PosVelAccYaw, "command/pos_vel_acc_yaw", 1)
        self._pta_pub = self._node.create_publisher(PoseTwistAccelCommand, "command/pose_twist_accel", 1)
        self._odom_sub = self._node.create_subscription(Odometry, "odom", self._odom_cb, 1)

        # Service
        self._set_arm_sc = self._node.create_client(SetArm, "set_arm")

    def _get_params(self) -> None:
        self._x_min = (
            self._node.declare_parameter("pose_limit/x/min", self.DEFAULT_MIN_X).get_parameter_value().double_value
        )
        self._x_max = (
            self._node.declare_parameter("pose_limit/x/max", self.DEFAULT_MAX_X).get_parameter_value().double_value
        )
        self._y_min = (
            self._node.declare_parameter("pose_limit/y/min", self.DEFAULT_MIN_Y).get_parameter_value().double_value
        )
        self._y_max = (
            self._node.declare_parameter("pose_limit/y/max", self.DEFAULT_MAX_Y).get_parameter_value().double_value
        )
        self._z_min = (
            self._node.declare_parameter("pose_limit/z/min", self.DEFAULT_MIN_Z).get_parameter_value().double_value
        )
        self._z_max = (
            self._node.declare_parameter("pose_limit/z/max", self.DEFAULT_MAX_Z).get_parameter_value().double_value
        )
        self._roll_min = (
            self._node.declare_parameter("pose_limit/roll/min", self.DEFAULT_MIN_ROLL)
            .get_parameter_value()
            .double_value
        )
        self._roll_max = (
            self._node.declare_parameter("pose_limit/roll/max", self.DEFAULT_MAX_ROLL)
            .get_parameter_value()
            .double_value
        )
        self._pitch_min = (
            self._node.declare_parameter("pose_limit/pitch/min", self.DEFAULT_MIN_PITCH)
            .get_parameter_value()
            .double_value
        )
        self._pitch_max = (
            self._node.declare_parameter("pose_limit/pitch/max", self.DEFAULT_MAX_PITCH)
            .get_parameter_value()
            .double_value
        )
        self._yaw_min = (
            self._node.declare_parameter("pose_limit/yaw/min", self.DEFAULT_MIN_YAW).get_parameter_value().double_value
        )
        self._yaw_max = (
            self._node.declare_parameter("pose_limit/yaw/max", self.DEFAULT_MAX_YAW).get_parameter_value().double_value
        )
        self._init_elevation = (
            self._node.declare_parameter("initial_elevation", self.DEFAULT_INIT_ELEVATION)
            .get_parameter_value()
            .double_value
        )

        assert self._x_min <= 0.0 <= self._x_max
        assert self._y_min <= 0.0 <= self._y_max
        assert self._z_min <= self._z_max
        assert self._roll_min <= 0.0 <= self._roll_max
        assert self._pitch_min <= 0.0 <= self._pitch_max
        assert self._yaw_min <= 0.0 <= self._yaw_max
        assert self._init_elevation >= 0.0

    def _set_arm(self, arming: bool) -> bool:
        if not self._set_arm_sc.service_is_ready():
            self._node.get_logger().error("Set-Arm service is not ready.")
            return False

        req = SetArm.Request()
        req.arming = arming

        res: SetArm.Response = self._set_arm_sc.call(req)
        if not res.success:
            self._node.get_logger().error(f"Failed to arm rotors: {res.message}")
            return False

        return True

    def _odom_cb(self, odom: Odometry) -> None:
        if odom.status != Odometry.NO_ERROR:
            return

        # Arming
        self._node.get_logger().info("Arming")
        if not self._set_arm(True):
            rclpy.sleep(self.ARM_RETRY_INTERVAL)
            return

        # 初期コマンドを設定
        self._cmd_x.set_value(odom.frame.trans.x)
        self._cmd_y.set_value(odom.frame.trans.y)
        self._cmd_z.set_value(odom.frame.trans.z + self._init_elevation)
        self._cmd_roll.set_value(0.0)
        self._cmd_pitch.set_value(0.0)
        self._cmd_yaw.set_value(euler_from_matrix(odom.frame.rot.data)[2])

        # バーを有効化
        self._cmd_x.setEnabled(True)
        self._cmd_y.setEnabled(True)
        self._cmd_z.setEnabled(True)
        self._cmd_roll.setEnabled(True)
        self._cmd_pitch.setEnabled(True)
        self._cmd_yaw.setEnabled(True)

        # 1回きりで終了
        self._odom_sub.unregister()

        self._node.get_logger().info("GUI teleoperation is ready.")

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
