from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .urdf_parser import URDFParser

import rclpy
from overrides import override
from PyQt5.QtWidgets import QHBoxLayout
from joint_state_publisher import JointStatePublisher
from joint_state_publisher_gui import JointStatePublisherGui

from tobas_std_tools_py.threading import KillableThread
from tobas_rqt_tools.widgets import Widget
from tobas_rqt_tools.roslaunch import rosrun

from .frame_tree import FrameTreeWidget
from .rviz import RvizWidget


class RobotVisualizerWidget(Widget):
    HEIGHT = 350
    JSP_WIDTH = 200

    def __init__(self, urdf_parser: URDFParser) -> None:
        super().__init__()

        self._jsp_gui = None
        self._jsp_thread = None
        self._rsp_process = None
        self._js2drs_process = None

        self._rviz = RvizWidget(urdf_parser)
        self._frame_tree = FrameTreeWidget(urdf_parser, self._rviz)

        # Layout
        self.setFixedHeight(self.HEIGHT)
        self._cols = QHBoxLayout()
        self.setLayout(self._cols)
        self._cols.addWidget(self._frame_tree)
        self._cols.addWidget(self._rviz)

    @override
    def close(self) -> bool:
        self._terminate_backgrounds()
        return super().close()

    def update_internal_data_structures(self) -> None:
        self._frame_tree.update_internal_data_structures()
        self._rviz.update_internal_data_structures()

        self._terminate_backgrounds()

        # Robot State Publisherを別プロセスで起動
        # Arrow等の表示に必要なTFを発行する役割
        # robot_descriptionがrosparamに登録された後に立ち上げる必要がある
        self._rsp_process = rosrun("robot_state_publisher", "robot_state_publisher")

        # JointState -> DisplayRobotStateの変換ノードを別プロセスで起動
        self._js2drs_process = rosrun("tobas_setup_assistant", "js2drs_node.py")

        # Joint State Publisherを別スレッドで起動
        jsp = JointStatePublisher()
        self._jsp_thread = KillableThread(target=jsp.loop)
        self._jsp_thread.start()

        # Joint State Publisher GUIを追加
        self._jsp_gui = JointStatePublisherGui("Joint States", jsp)
        self._jsp_gui.setFixedWidth(self.JSP_WIDTH)
        self._cols.addWidget(self._jsp_gui)

    def highlight_link(self, link_name: str) -> None:
        return self._rviz.highlight_link(link_name)

    def _terminate_backgrounds(self) -> None:
        if self._jsp_gui is not None:
            # Joint State Publisher GUIを削除
            self._cols.removeWidget(self._jsp_gui)

            # バックグラウンドのスレッドを終了
            if not self._jsp_thread.kill():
                rclpy.logwarn("Failed to kill the thread of joint state publisher.")

            # バックグラウンドのプロセスを終了
            self._rsp_process.terminate()
            self._js2drs_process.terminate()
