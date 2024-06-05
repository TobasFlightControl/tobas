import rospy
from typing import Dict
from functools import partial
from urdf_parser_py.urdf import Robot, Joint, JointLimit
from sensor_msgs.msg import JointState
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QPushButton, QVBoxLayout

from tobas_rqt_tools.widgets import Widget, FloatSliderDisplay

from .common import BUTTON_HEIGHT


class JointPositionsCommanderWidget(Widget):
    POSITION = "position"
    VELOCITY = "velocity"
    EFFORT = "effort"

    PUBILSH_CMDS_TIMER_PERIOD = 0.1  # [s]

    def __init__(self) -> None:
        super().__init__()

        # rosparams
        self._home_positions: Dict[str, float] = {}
        self._cmd_types: Dict[str, str] = {}
        self._get_params()

        # コマンド
        self._tar_js_pos = JointState()
        self._tar_js_vel = JointState()
        self._tar_js_eff = JointState()
        for jnt_name, cmd_type in self._cmd_types.items():
            home_pos = self._home_positions[jnt_name]
            if cmd_type == self.POSITION:
                self._tar_js_pos.name.append(jnt_name)
                self._tar_js_pos.position.append(home_pos)
            elif cmd_type == self.VELOCITY:
                self._tar_js_vel.name.append(jnt_name)
                self._tar_js_vel.position.append(home_pos)
            elif cmd_type == self.EFFORT:
                self._tar_js_eff.name.append(jnt_name)
                self._tar_js_eff.position.append(home_pos)
                self._tar_js_eff.velocity.append(0.0)
            else:
                raise RuntimeError(f"Unknown joint command type: {cmd_type}")

        # Publishers
        self._tar_pos_pub = rospy.Publisher("joint_position_controller/target_joint_states", JointState, queue_size=1)
        self._tar_js_vel_pub = rospy.Publisher(
            "joint_velocity_controller/target_joint_states", JointState, queue_size=1
        )
        self._tar_js_eff_pub = rospy.Publisher("joint_effort_controller/target_joint_states", JointState, queue_size=1)

        # メインレイアウト
        rows = QVBoxLayout()
        self.setLayout(rows)

        # Commandersをセット
        robot: Robot = Robot.from_parameter_server("robot_description")
        self._commanders: Dict[str, FloatSliderDisplay] = {}
        for jnt_name in self._cmd_types.keys():
            joint: Joint = robot.joint_map[jnt_name]
            limit: JointLimit = joint.limit
            commander = FloatSliderDisplay()
            commander.set_text(jnt_name)
            commander.set_minimum(limit.lower)
            commander.set_maximum(limit.upper)
            commander.set_value(self._home_positions[jnt_name])
            commander.set_callback(partial(self._on_value_changed, jnt_name=jnt_name))
            self._commanders[jnt_name] = commander
            rows.addWidget(commander)

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

        self._publish_commands_timer = rospy.Timer(
            rospy.Duration(self.PUBILSH_CMDS_TIMER_PERIOD), self._publish_commands_timer_cb
        )

    def _get_params(self) -> None:
        num_joints = rospy.get_param("num_joints")
        for i in range(num_joints):
            jnt_name = rospy.get_param(f"joint_{i}/name")
            home_pos = rospy.get_param(f"joint_{i}/home_position")
            cmd_type = rospy.get_param(f"joint_{i}/command_type")
            self._home_positions[jnt_name] = home_pos
            self._cmd_types[jnt_name] = cmd_type

    def _publish_current_commands(self) -> None:
        self._tar_pos_pub.publish(self._tar_js_pos)
        self._tar_js_vel_pub.publish(self._tar_js_vel)
        self._tar_js_eff_pub.publish(self._tar_js_eff)

    def _publish_commands_timer_cb(self, _) -> None:
        self._publish_current_commands()

    @pyqtSlot(float)
    def _on_value_changed(self, value: float, jnt_name: str) -> None:
        cmd_type = self._cmd_types[jnt_name]

        if cmd_type == self.POSITION:
            idx = self._tar_js_pos.name.index(jnt_name)
            self._tar_js_pos.position[idx] = value
            self._tar_pos_pub.publish(self._tar_js_pos)
        elif cmd_type == self.VELOCITY:
            idx = self._tar_js_vel.name.index(jnt_name)
            self._tar_js_vel.position[idx] = value
            self._tar_js_vel_pub.publish(self._tar_js_vel)
        elif cmd_type == self.EFFORT:
            idx = self._tar_js_eff.name.index(jnt_name)
            self._tar_js_eff.position[idx] = value
            self._tar_js_eff_pub.publish(self._tar_js_eff)
        else:
            raise RuntimeError(f"Unknown joint command type: {cmd_type}")

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
        self.setEnabled(False)

        for joint_cmd in self._commanders.values():
            joint_cmd.set_random_value()

        self.setEnabled(True)
