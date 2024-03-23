from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

from threading import Thread
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from joint_state_publisher import JointStatePublisher
from joint_state_publisher_gui import JointStatePublisherGui

from tobas_std_tools_py.threading import KillableThread
from tobas_rqt_tools.roslaunch import rosrun

from .frame_tree import FrameTreeWidget
from .rviz import RvizWidget


class RobotVisualizerWidget(QWidget):
    HEIGHT = 350
    JSP_WIDTH = 200

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._jsp_gui = None
        self._jsp_thread = None

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
        self._main.urdf_parser.robot_model_updated.connect(self._on_robot_model_updated)

    def highlight_link(self, link_name: str) -> None:
        return self._rviz.highlight_link(link_name)

    @pyqtSlot()
    def _on_robot_model_updated(self) -> None:
        if self._jsp_gui is not None:
            # Joint State Publisher GUIを削除
            self._cols.removeWidget(self._jsp_gui)

            # バックグラウンドのスレッドをキル
            self._jsp_thread.kill()

        # Robot State Publisherを別スレッドで起動
        # Arrow等の表示に必要なTFを発行する役割
        # robot_descriptionがrosparamに登録された後に立ち上げる必要がある
        Thread(target=lambda: rosrun("robot_state_publisher", "robot_state_publisher")).start()

        # JointState -> DisplayRobotStateの変換ノードを別スレッドで起動
        # ROSノードは同名があれば自動でシャットダウンされるため，手動でスレッドをキルする必要はない
        Thread(target=lambda: rosrun("tobas_setup_assistant", "js2drs_node.py")).start()

        # Joint State Publisherを別スレッドで起動
        jsp = JointStatePublisher()
        self._jsp_thread = KillableThread(target=jsp.loop)
        self._jsp_thread.start()

        # Joint State Publisher GUIを追加
        self._jsp_gui = JointStatePublisherGui("Joint States", jsp)
        self._jsp_gui.setFixedWidth(self.JSP_WIDTH)
        self._cols.addWidget(self._jsp_gui)

        self.setVisible(True)
