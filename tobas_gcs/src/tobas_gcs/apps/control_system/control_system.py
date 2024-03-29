from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.widgets import Widget

from .actions_commander import ActionsCommanderWidget
from .pose_viewer import PoseViewerWidget
from .twist_viewer import TwistViewerWidget
from .battery_viewer import BatteryViewerWidget
from .rotors_viewer import RotorsViewerWidget
from .status_viewer import StatusViewerWidget
from .rc_input_viewer import RCInputViewerWidget


class ControlSystemWidget(Widget):
    NAME = "Control System"

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__(parent=main)
        self._main = main

        rows = QVBoxLayout()
        self.setLayout(rows)

        self._actions_commander = ActionsCommanderWidget(main)
        self._pose_viewer = PoseViewerWidget(main)
        self._twist_viewer = TwistViewerWidget(main)
        self._battery_viewer = BatteryViewerWidget(main)
        self._rotors_viewer = RotorsViewerWidget(main)
        self._status_viewer = StatusViewerWidget(main)
        self._rc_input_viewer = RCInputViewerWidget(main)

        rows.addWidget(self._actions_commander)
        rows.addWidget(self._pose_viewer)
        rows.addWidget(self._twist_viewer)
        rows.addWidget(self._battery_viewer)
        rows.addWidget(self._rotors_viewer)
        rows.addWidget(self._status_viewer)
        rows.addWidget(self._rc_input_viewer)

        rows.addStretch()

    def define_connections(self) -> None:
        self._actions_commander.define_connections()
        self._pose_viewer.define_connections()
        self._twist_viewer.define_connections()
        self._battery_viewer.define_connections()
        self._rotors_viewer.define_connections()
        self._status_viewer.define_connections()
        self._rc_input_viewer.define_connections()
