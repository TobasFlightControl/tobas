import os.path as osp
import rospy
from typing import List
from functools import partial
from urdf_parser_py.urdf import Robot, Joint, JointLimit
from sensor_msgs.msg import JointState
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import MainWidget, FloatSliderDisplay, add_spacer
from dh_rqt_tools.path import get_proj_path

from .common import *


class JointPositionsCommander(MainWidget):
    def __init__(self) -> None:
        super().__init__()

        proj_path = get_proj_path()
        icon_path = osp.join(proj_path, "resources/icon.png")
        self.setWindowIcon(QIcon(icon_path))
        self.setWindowTitle("Joint State Commander")

        # rosparams
        self._joint_names: List[str] = []
        self._get_params()

        # The number of joints of which command is published
        nj = len(self._joint_names)

        # コマンド
        self._cmd = JointState()
        self._cmd.name = sorted(self._joint_names)
        self._cmd.position = [0] * nj
        self._cmd.velocity = [0] * nj
        self._cmd.effort = [0] * nj

        # Publisher
        self._cmd_pub = rospy.Publisher(
            "command/joint_states", JointState, queue_size=1
        )

        # メインレイアウト
        rows = QVBoxLayout()
        self.setLayout(rows)

        # Commandersをセット
        robot: Robot = Robot.from_parameter_server("robot_description")
        self._commanders: List[FloatSliderDisplay] = []
        for i, jnt_name in enumerate(self._cmd.name):
            joint: Joint = robot.joint_map[jnt_name]
            limit: JointLimit = joint.limit
            commander = FloatSliderDisplay(
                jnt_name,
                limit.lower,
                limit.upper,
                callback=partial(self._on_value_changed, idx=i),
            )
            commander.update()
            self._commanders.append(commander)
            rows.addWidget(commander)

        self._home_button = QPushButton("Home", parent=self)
        self._home_button.setFixedHeight(BUTTON_HEIGHT)
        self._home_button.clicked.connect(self._on_home_button_clicked)
        rows.addWidget(self._home_button)

        self._center_button = QPushButton("Center", parent=self)
        self._center_button.setFixedHeight(BUTTON_HEIGHT)
        self._center_button.clicked.connect(self._on_center_button_clicked)
        rows.addWidget(self._center_button)

        self._random_button = QPushButton("Randomize", parent=self)
        self._random_button.setFixedHeight(BUTTON_HEIGHT)
        self._random_button.clicked.connect(self._on_random_button_clicked)
        rows.addWidget(self._random_button)

        add_spacer(rows)

    def _get_params(self) -> None:
        self._joint_names = rospy.get_param("posture_defining_joint_names")

    @pyqtSlot(float)
    def _on_value_changed(self, value: float, idx: int) -> None:
        self._cmd.position[idx] = value
        self._cmd_pub.publish(self._cmd)

    @pyqtSlot()
    def _on_home_button_clicked(self) -> None:
        """全ての関節角をホームポジションに設定する．"""
        self.setEnabled(False)

        for joint_cmd in self._commanders:
            joint_cmd.set_value(0.0)  # TODO: 0以外がホームポジションになり得る？

        self.setEnabled(True)

    @pyqtSlot()
    def _on_center_button_clicked(self) -> None:
        """全ての関節角を中央の値に設定する．"""
        self.setEnabled(False)

        for joint_cmd in self._commanders:
            joint_cmd.set_center_value()

        self.setEnabled(True)

    @pyqtSlot()
    def _on_random_button_clicked(self) -> None:
        """全ての関節角をランダム値に設定する．"""
        self.setEnabled(False)

        for joint_cmd in self._commanders:
            joint_cmd.set_random_value()

        self.setEnabled(True)
