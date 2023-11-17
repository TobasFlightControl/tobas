import os.path as osp
import rospy
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import MainWidget, add_expanding_widget
from dh_rqt_tools.path import get_proj_path

from .common import *
from .commanders import MultirotorCommanderWidget, JointPositionCommanderWidget


class GuiTeleopWidget(MainWidget):
    WAIT_TO_CONNECT = 0.5  # [s]

    def __init__(self) -> None:
        super().__init__()

        proj_path = get_proj_path()
        icon_path = osp.join(proj_path, "resources/icon.png")
        self.setWindowIcon(QIcon(icon_path))
        self.setWindowTitle("Multirotor GUI Teleop")

        rows = QVBoxLayout()
        self.setLayout(rows)

        self._drone_commander = MultirotorCommanderWidget(self)
        rows.addWidget(self._drone_commander)

        self._joint_commander = JointPositionCommanderWidget(self)
        rows.addWidget(self._joint_commander)

        add_expanding_widget(rows)

        # 接続が完了するまで少し待ってから全ての関節値を発行
        rospy.sleep(self.WAIT_TO_CONNECT)
        self._drone_commander.publish_current_command()
        self._joint_commander.publish_current_command()
