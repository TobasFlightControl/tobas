from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

import os.path as osp
import rospkg
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import create_rviz_frame

from .common import *


class RvizWidget(QWidget):
    MIN_WIDTH = 300
    DEFAULT_VISUAL_ENABLED = True
    DEFAULT_COLLISION_ENABLED = False

    def __init__(self, main: SetupAssistant):
        super().__init__()
        self._main = main

        self._highlighted_link = ""

        # Create Rviz frame widget
        pkg_path = rospkg.RosPack().get_path(PKG_NAME)
        rviz_config_path = osp.join(pkg_path, "config/setup_assistant.rviz")
        self._frame = create_rviz_frame(rviz_config_path)

        # Setup robot_model_display
        # rviz::Display Class Reference: https://docs.ros.org/en/diamondback/api/rviz/html/classrviz_1_1Display.html
        manager = self._frame.getManager()
        self._display = manager.getRootDisplayGroup().getDisplayAt(0)
        assert self._display.getName() == "RobotState"
        self._display.setBool(False)

        # ハイライトプロパティ
        self._highlight_link = self._display.subProp("Highlight Link")
        self._unhighlight_link = self._display.subProp("Unhighlight Link")

        # 可視化プロパティ
        self._enable_visual = self._display.subProp("Visual Enabled")
        self._enable_collision = self._display.subProp("Collision Enabled")
        self._enable_visual.setBool(self.DEFAULT_VISUAL_ENABLED)
        self._enable_collision.setBool(self.DEFAULT_COLLISION_ENABLED)

        # 可視化ボタン
        self._visual_box = QCheckBox("Visual Enabled")
        self._visual_box.setChecked(self.DEFAULT_VISUAL_ENABLED)
        self._collision_box = QCheckBox("Collision Enabled")
        self._collision_box.setChecked(self.DEFAULT_COLLISION_ENABLED)

        # レイアウト
        rows = QVBoxLayout()
        cols = QHBoxLayout()
        self.setLayout(rows)
        rows.addWidget(self._frame)
        rows.addLayout(cols)
        cols.addStretch()
        cols.addWidget(self._visual_box)
        cols.addWidget(self._collision_box)

        self.setMinimumWidth(self.MIN_WIDTH)

    def define_connections(self) -> None:
        self._main.urdf_parser.robot_model_loaded.connect(self._on_robot_model_loaded)
        self._visual_box.toggled.connect(self._on_visual_box_toggled)
        self._collision_box.toggled.connect(self._on_collision_box_toggled)

    def highlight_link(self, link_name: str) -> None:
        if link_name == self._highlighted_link:
            return

        if self._highlighted_link:
            self.unhighlight_link(self._highlighted_link)

        self._highlight_link.setValue(link_name)
        self._highlighted_link = link_name

    def unhighlight_link(self, link_name: str) -> None:
        self._unhighlight_link.setValue(link_name)

    @pyqtSlot()
    def _on_robot_model_loaded(self) -> None:
        root_link = self._main.urdf_parser.get_root()
        self._frame.getManager().setFixedFrame(root_link.name)
        self._display.setBool(True)

    @pyqtSlot(bool)
    def _on_visual_box_toggled(self, checked: bool) -> None:
        self._enable_visual.setBool(checked)

    @pyqtSlot(bool)
    def _on_collision_box_toggled(self, checked: bool) -> None:
        self._enable_collision.setBool(checked)
