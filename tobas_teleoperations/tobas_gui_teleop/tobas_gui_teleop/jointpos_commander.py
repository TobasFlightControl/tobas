from typing import Dict
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import JointState

from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QPushButton, QVBoxLayout

from tobas_rqt_py.widgets import Widget, FloatSliderDisplay
from tobas_drone_msgs.msg import Drone

from .common import BUTTON_HEIGHT


class JointPositionsCommanderWidget(Widget):
    POSITION = 0
    VELOCITY = 1
    EFFORT = 2

    PUBILSH_CMDS_PERIOD = 0.1  # [s]

    def __init__(self, node: Node) -> None:
        super().__init__()
        self._node = node

        # QoS
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        latch_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Droneを取得
        self._node.get_logger().info("Waiting for drone configuration.")
        res = wait_for_message(Drone, self._node, "remote_interface/drone", qos_profile=latch_qos, time_to_wait=5)
        success: bool = res[0]
        drone: Drone = res[1]
        if not success:
            self._node.get_logger().error("Failed to get drone configuration from topic.")
            rclpy.shutdown()
            return

        # メインレイアウト
        rows = QVBoxLayout()
        self.setLayout(rows)

        # ジョイントの情報を取得
        self._home_positions: Dict[str, float] = {}
        self._interfaces: Dict[str, int] = {}
        self._commanders: Dict[str, FloatSliderDisplay] = {}
        self._tar_js_pos = JointState()
        self._tar_js_vel = JointState()
        self._tar_js_eff = JointState()

        for joint in drone.joints:
            self._home_positions[joint.name] = joint.home_pos
            self._interfaces[joint.name] = joint.interface

            commander = FloatSliderDisplay()
            commander.set_text(joint.name)
            commander.set_minimum(joint.min_pos)
            commander.set_maximum(joint.max_pos)
            commander.set_value(self._home_positions[joint.name])
            commander.set_callback(partial(self._on_value_changed, jnt_name=joint.name))
            self._commanders[joint.name] = commander
            rows.addWidget(commander)

            match joint.interface:
                case self.POSITION:
                    self._tar_js_pos.name.append(joint.name)
                    self._tar_js_pos.position.append(joint.home_pos)
                case self.VELOCITY:
                    self._tar_js_vel.name.append(joint.name)
                    self._tar_js_vel.position.append(joint.home_pos)
                case self.EFFORT:
                    self._tar_js_eff.name.append(joint.name)
                    self._tar_js_eff.position.append(joint.home_pos)
                    self._tar_js_eff.velocity.append(0.0)
                case _:
                    raise RuntimeError(f"Unknown joint command type: {joint.interface}")

        self._home_button = QPushButton("Home")
        self._home_button.setFixedHeight(BUTTON_HEIGHT)
        self._home_button.clicked.connect(self._on_home_button_clicked)
        rows.addWidget(self._home_button)

        self._center_button = QPushButton("Center")
        self._center_button.setFixedHeight(BUTTON_HEIGHT)
        self._center_button.clicked.connect(self._on_center_button_clicked)
        rows.addWidget(self._center_button)

        self._random_button = QPushButton("Randomize")
        self._random_button.setFixedHeight(BUTTON_HEIGHT)
        self._random_button.clicked.connect(self._on_random_button_clicked)
        rows.addWidget(self._random_button)

        rows.addStretch()

        # Publishers
        self._tar_pos_pub = self._node.create_publisher(
            JointState, "joint_position_controller/target_joint_states", cmd_qos
        )
        self._tar_js_vel_pub = self._node.create_publisher(
            JointState, "joint_velocity_controller/target_joint_states", cmd_qos
        )
        self._tar_js_eff_pub = self._node.create_publisher(
            JointState, "joint_effort_controller/target_joint_states", cmd_qos
        )

        # Timers
        self._publish_cmds_timer = self._node.create_timer(self.PUBILSH_CMDS_PERIOD, self._publish_commands_timer_cb)

    def _publish_current_commands(self) -> None:
        self._tar_pos_pub.publish(self._tar_js_pos)
        self._tar_js_vel_pub.publish(self._tar_js_vel)
        self._tar_js_eff_pub.publish(self._tar_js_eff)

    def _publish_commands_timer_cb(self) -> None:
        self._publish_current_commands()

    @pyqtSlot(float)
    def _on_value_changed(self, value: float, jnt_name: str) -> None:
        interface = self._interfaces[jnt_name]

        match interface:
            case self.POSITION:
                idx = self._tar_js_pos.name.index(jnt_name)
                self._tar_js_pos.position[idx] = value
                self._tar_pos_pub.publish(self._tar_js_pos)
            case self.VELOCITY:
                idx = self._tar_js_vel.name.index(jnt_name)
                self._tar_js_vel.position[idx] = value
                self._tar_js_vel_pub.publish(self._tar_js_vel)
            case self.EFFORT:
                idx = self._tar_js_eff.name.index(jnt_name)
                self._tar_js_eff.position[idx] = value
                self._tar_js_eff_pub.publish(self._tar_js_eff)
            case _:
                raise RuntimeError(f"Unknown joint interface: {interface}")

    @pyqtSlot()
    def _on_home_button_clicked(self) -> None:
        """全ての関節角をホームポジションに設定する．"""
        for jnt_name, home_pos in self._home_positions.items():
            self._commanders[jnt_name].set_value(home_pos)

        self._publish_current_commands()

    @pyqtSlot()
    def _on_center_button_clicked(self) -> None:
        """全ての関節角を中央の値に設定する．"""
        for joint_cmd in self._commanders.values():
            joint_cmd.set_center_value()

        self._publish_current_commands()

    @pyqtSlot()
    def _on_random_button_clicked(self) -> None:
        """全ての関節角をランダム値に設定する．"""
        for joint_cmd in self._commanders.values():
            joint_cmd.set_random_value()

        self._publish_current_commands()
