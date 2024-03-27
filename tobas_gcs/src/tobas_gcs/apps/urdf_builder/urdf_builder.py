from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

import os.path as osp
import rospkg
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import Widget, create_rviz_frame


class UrdfBuilderWidget(Widget):
    NAME = "URDF Builder"

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__(parent=main)
        self._main = main

        rows = QVBoxLayout()
        self.setLayout(rows)

        pkg_path = rospkg.RosPack().get_path("urdf_builder")
        rviz_config_path = osp.join(pkg_path, "config/urdf_builder.rviz")
        self._frame = create_rviz_frame(rviz_config_path)

        rows.addWidget(self._frame)

    def define_connections(self) -> None:
        pass
