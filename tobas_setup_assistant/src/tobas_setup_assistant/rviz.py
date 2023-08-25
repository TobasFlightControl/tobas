from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

from dh_rqt_tools.path import get_proj_path

import os.path as osp
from rviz import bindings as rviz
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


class RvizWidget(QWidget):

    MIN_WIDTH = 300

    def __init__(self, main: SetupAssistant):
        super().__init__()
        self._main = main

        self._highlighted_link = ""

        # Setup frame
        # cf. RViz Python Tutorial: https://docs.ros.org/en/indigo/api/rviz_python_tutorial/html/
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        rviz_config_path = osp.join(get_proj_path(), "config/setup_assistant.rviz")
        reader.readFile(config, rviz_config_path)

        self._frame = rviz.VisualizationFrame()
        self._frame.setSplashPath("")
        self._frame.initialize()
        self._frame.load(config)
        self._frame.setMenuBar(None)
        self._frame.setStatusBar(None)
        self._frame.setHideButtonVisibility(False)

        # Setup robot_model_display
        manager = self._frame.getManager()
        self._display = manager.getRootDisplayGroup().getDisplayAt(2)
        self._display.setBool(False)

        # robot_model_displayのサブプロパティを取得
        self._highlight_link = self._display.subProp("Highlight Link")
        self._unhighlight_link = self._display.subProp("Unhighlight Link")

        # Layout
        self._rows = QVBoxLayout()
        self._rows.addWidget(self._frame)
        self.setLayout(self._rows)

        self.setMinimumWidth(self.MIN_WIDTH)

    def define_connections(self) -> None:
        self._main.urdf_parser.robot_model_updated.connect(self._on_robot_model_updated)

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
    def _on_robot_model_updated(self) -> None:
        root_link = self._main.urdf_parser.get_root()
        self._frame.getManager().setFixedFrame(root_link.name)
        self._display.setBool(True)
