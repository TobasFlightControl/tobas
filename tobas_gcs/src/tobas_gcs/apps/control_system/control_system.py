from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import Widget
from tobas_tools_py.drone import Drone

from .actions_commander import ActionsCommanderWidget
from .pose_viewer import PoseViewerWidget
from .twist_viewer import TwistViewerWidget
from .battery_viewer import BatteryViewerWidget
from .rotors_viewer import RotorsViewerWidget
from .status_viewer import StatusViewerWidget
from .rc_input_viewer import RCInputViewerWidget

from ..base import BaseAppWidget


class ControlSystemWidget(BaseAppWidget):
    NAME = "Control System"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        rows = QVBoxLayout()
        self.setLayout(rows)

        self._actions_commander = ActionsCommanderWidget(main, drone)
        self._pose_viewer = PoseViewerWidget(main, drone)
        self._twist_viewer = TwistViewerWidget(main, drone)
        self._battery_viewer = BatteryViewerWidget(main, drone)
        self._rotors_viewer = RotorsViewerWidget(main, drone)
        self._status_viewer = StatusViewerWidget(main, drone)
        self._rc_input_viewer = RCInputViewerWidget(main, drone)

        rows.addWidget(self._actions_commander)
        rows.addWidget(self._pose_viewer)
        rows.addWidget(self._twist_viewer)
        rows.addWidget(self._battery_viewer)
        rows.addWidget(self._rotors_viewer)
        rows.addWidget(self._status_viewer)
        rows.addWidget(self._rc_input_viewer)

        rows.addStretch()

    @override
    def define_connections(self) -> None:
        self._actions_commander.define_connections()
        self._pose_viewer.define_connections()
        self._twist_viewer.define_connections()
        self._battery_viewer.define_connections()
        self._rotors_viewer.define_connections()
        self._status_viewer.define_connections()
        self._rc_input_viewer.define_connections()
