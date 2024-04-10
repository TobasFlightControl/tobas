from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.layouts import ScrollableVBoxLayout
from tobas_tools_py.drone import Drone

from ..base import BaseAppWidget
from .sections import *


class ControlSystemWidget(BaseAppWidget):
    NAME = "Control System"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        rows = ScrollableVBoxLayout()
        self.setLayout(rows)

        self._actions_commander = ActionsCommanderWidget(main, drone)
        self._position_viewer = PositionViewerWidget(main, drone)
        self._orientation_viewer = OrientationViewerWidget(main, drone)
        self._velocity_viewer = VelocityViewerWidget(main, drone)
        self._battery_viewer = BatteryViewerWidget(main, drone)
        self._rotors_viewer = RotorsViewerWidget(main, drone)
        self._status_viewer = StatusViewerWidget(main, drone)
        self._rc_input_viewer = RCInputViewerWidget(main, drone)
        self._cpu_viewer = CpuViewerWidget(main, drone)
        self._latency_viewer = LatencyViewerWidget(main, drone)

        rows.addWidget(self._actions_commander)
        rows.addWidget(self._position_viewer)
        rows.addWidget(self._orientation_viewer)
        # rows.addWidget(self._velocity_viewer)  # TODO
        rows.addWidget(self._battery_viewer)
        # rows.addWidget(self._rotors_viewer)  # TODO
        rows.addWidget(self._status_viewer)
        rows.addWidget(self._rc_input_viewer)
        rows.addWidget(self._cpu_viewer)
        rows.addWidget(self._latency_viewer)

        rows.addStretch()

    @override
    def define_connections(self) -> None:
        self._actions_commander.define_connections()
        self._position_viewer.define_connections()
        self._orientation_viewer.define_connections()
        self._velocity_viewer.define_connections()
        self._battery_viewer.define_connections()
        self._rotors_viewer.define_connections()
        self._status_viewer.define_connections()
        self._rc_input_viewer.define_connections()
        self._cpu_viewer.define_connections()
        self._latency_viewer.define_connections()

    @override
    def update_internal_data_structures(self) -> None:
        self._actions_commander.update_internal_data_structures()
        self._position_viewer.update_internal_data_structures()
        self._orientation_viewer.update_internal_data_structures()
        self._velocity_viewer.update_internal_data_structures()
        self._battery_viewer.update_internal_data_structures()
        self._rotors_viewer.update_internal_data_structures()
        self._status_viewer.update_internal_data_structures()
        self._rc_input_viewer.update_internal_data_structures()
        self._cpu_viewer.update_internal_data_structures()
        self._latency_viewer.update_internal_data_structures()
