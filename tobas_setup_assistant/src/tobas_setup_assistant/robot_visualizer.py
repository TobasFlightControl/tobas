from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

import threading
import subprocess
import rospy
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from joint_state_publisher import JointStatePublisher
from joint_state_publisher_gui import JointStatePublisherGui

from dh_rqt_tools.messages import q_error

from .frame_tree import FrameTreeWidget
from .rviz import RvizWidget


class RobotVisualizerWidget(QWidget):
    HEIGHT = 350
    JSP_WIDTH = 200

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._jsp = None
        self._jsp_gui = None

        self._cols = QHBoxLayout()
        self.setLayout(self._cols)

        self._frame_tree = FrameTreeWidget(main)
        self._cols.addWidget(self._frame_tree)

        self._rviz = RvizWidget(main)
        self._cols.addWidget(self._rviz)

        self.setFixedHeight(self.HEIGHT)
        self.setVisible(False)

    def define_connections(self) -> None:
        self._frame_tree.define_connections()
        self._rviz.define_connections()
        self._main.urdf_parser.robot_model_loaded.connect(self._on_robot_model_loaded)

    def highlight_link(self, link_name: str) -> None:
        return self._rviz.highlight_link(link_name)

    @pyqtSlot()
    def _on_robot_model_loaded(self) -> None:
        # Robot State Publisherを別スレッドで起動
        threading.Thread(target=self._run_rsp).start()

        # Joint State Publisherを別スレッドで起動
        self._jsp = JointStatePublisher()
        threading.Thread(target=self._jsp.loop).start()

        # JointState -> DisplayRobotStateの変換ノードを別スレッドで起動
        threading.Thread(target=self._run_converter).start()

        self._jsp_gui = JointStatePublisherGui("Joint States", self._jsp)
        self._jsp_gui.setFixedWidth(self.JSP_WIDTH)
        self._cols.addWidget(self._jsp_gui)

        self.setVisible(True)

    @staticmethod
    def _run_rsp() -> None:
        try:
            subprocess.run(
                "rosrun robot_state_publisher robot_state_publisher",
                shell=True,
                check=True,
            )
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Failed to launch robot_state_publisher: {e}")

    @staticmethod
    def _run_converter() -> None:
        try:
            subprocess.run(
                "rosrun tobas_setup_assistant joint_state_to_display_robot_state_node.py",
                shell=True,
                check=True,
            )
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Failed to launch DisplayRobotState publisher: {e}")
