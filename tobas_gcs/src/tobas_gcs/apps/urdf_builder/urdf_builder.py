from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

import os.path as osp
import rospkg
from overrides import override
from PyQt5.QtWidgets import QVBoxLayout

from tobas_rqt_tools.rviz import create_rviz_frame
from tobas_tools_py.drone import Drone

from ..base import BaseAppWidget


class UrdfBuilderWidget(BaseAppWidget):
    NAME = "URDF Builder"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        rows = QVBoxLayout()
        self.setLayout(rows)

        pkg_path = rospkg.RosPack().get_path("urdf_builder")
        rviz_config_path = osp.join(pkg_path, "config/urdf_builder.rviz")
        self._frame = create_rviz_frame(rviz_config_path)

        rows.addWidget(self._frame)

    @override
    def update_internal_data_structures(self) -> None:
        pass
