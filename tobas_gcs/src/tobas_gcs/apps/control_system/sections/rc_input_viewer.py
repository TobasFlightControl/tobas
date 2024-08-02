from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import rclpy
from typing import override
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QLabel, QVBoxLayout, QHBoxLayout, QGridLayout

from tobas_rqt_tools.widgets import FramedLabel, HPositionBarWidget, VPositionBarWidget
from tobas_rqt_tools.utils import place_center, create_fixed_height_hboxlayout
from tobas_tools_py.constants import RCRange, Topic
from tobas_tools_py.drone import Drone
from tobas_msgs.msg import RCInput

from .base_section import BaseControlSystemSectionWidget


class RCInputViewerWidget(BaseControlSystemSectionWidget):
    LABEL = "Radio Input"

    RANGE_SIDE_SHORT = 30
    RANGE_SIDE_LONG = 300
    LABEL_WIDTH = 100
    LABEL_HEIGHT = 30

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        cols1 = QHBoxLayout()
        self._rows.addLayout(cols1)

        # Roll, Pitch, Yaw, Throttle
        cols2 = create_fixed_height_hboxlayout(self.RANGE_SIDE_LONG + 20, cols1)

        self._pitch_range = VPositionBarWidget(fill_range=False, minimum=RCRange.MAX, maximum=RCRange.MIN)
        self._pitch_range.setFixedSize(self.RANGE_SIDE_SHORT, self.RANGE_SIDE_LONG)
        cols2.addWidget(self._pitch_range)

        rows1 = QVBoxLayout()
        cols2.addLayout(rows1)

        self._roll_range = HPositionBarWidget(fill_range=False, minimum=RCRange.MIN, maximum=RCRange.MAX)
        self._roll_range.setFixedSize(self.RANGE_SIDE_LONG, self.RANGE_SIDE_SHORT)
        place_center(self._roll_range, rows1)
        place_center(QLabel(f"Roll"), rows1)

        rows1.addStretch()

        cols3 = QHBoxLayout()
        rows1.addLayout(cols3)

        pitch_label = QLabel(f"Pitch")
        pitch_label.setAlignment(Qt.AlignmentFlag.AlignLeft)
        cols3.addWidget(pitch_label)

        throttle_label = QLabel(f"Throttle")
        throttle_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        cols3.addWidget(throttle_label)

        rows1.addStretch()

        place_center(QLabel(f"Yaw"), rows1)
        self._yaw_range = HPositionBarWidget(fill_range=False, minimum=RCRange.MAX, maximum=RCRange.MIN)
        self._yaw_range.setFixedSize(self.RANGE_SIDE_LONG, self.RANGE_SIDE_SHORT)
        place_center(self._yaw_range, rows1)

        self._throttle_range = VPositionBarWidget(fill_range=False, minimum=RCRange.MAX, maximum=RCRange.MIN)
        self._throttle_range.setFixedSize(self.RANGE_SIDE_SHORT, self.RANGE_SIDE_LONG)
        cols2.addWidget(self._throttle_range)

        cols1.addSpacing(30)

        bar_grid = QGridLayout()
        cols1.addLayout(bar_grid)

        # Mode
        bar_grid.addWidget(QLabel(f"Mode  :"), 0, 0)
        self._mode = FramedLabel()  # QLineEditだと処理が重すぎるのか落ちてしまう
        self._mode.setFixedSize(self.LABEL_WIDTH, self.LABEL_HEIGHT)
        bar_grid.addWidget(self._mode, 0, 1)

        # E-Stop
        bar_grid.addWidget(QLabel(f"E-Stop:"), 1, 0)
        self._estop = FramedLabel()
        self._estop.setFixedSize(self.LABEL_WIDTH, self.LABEL_HEIGHT)
        bar_grid.addWidget(self._estop, 1, 1)

        # GPSw
        bar_grid.addWidget(QLabel(f"GPSw  :"), 2, 0)
        self._gpsw = FramedLabel()
        self._gpsw.setFixedSize(self.LABEL_WIDTH, self.LABEL_HEIGHT)
        bar_grid.addWidget(self._gpsw, 2, 1)

        cols1.addStretch()

        # Subscriber
        self._rcin_sub = None

    @override
    def update_internal_data_structures(self) -> None:
        # Clear
        self._roll_range.clear()
        self._pitch_range.clear()
        self._yaw_range.clear()
        self._throttle_range.clear()
        self._mode.clear()
        self._estop.clear()
        self._gpsw.clear()

        # Update subscriber
        if self._rcin_sub is not None:
            self._rcin_sub.unregister()
        self._rcin_sub = rclpy.Subscriber(
            f"{self._drone.name}/{Topic.Throttled.RC_INPUT}",
            RCInput,
            self._rcin_cb,
            queue_size=1,
        )

    def _rcin_cb(self, rcin: RCInput) -> None:
        self._roll_range.set_value(rcin.roll)
        self._pitch_range.set_value(rcin.pitch)
        self._yaw_range.set_value(rcin.yaw)
        self._throttle_range.set_value(rcin.throttle)

        self._roll_range.update()
        self._pitch_range.update()
        self._yaw_range.update()
        self._throttle_range.update()

        if rcin.mode == RCInput.MODE_PROGRAM:
            self._mode.setText("Program")
        elif rcin.mode == RCInput.MODE_STABILIZE:
            self._mode.setText("Stabilize")
        elif rcin.mode == RCInput.MODE_ACROBAT:
            self._mode.setText("Acrobat")
        else:
            self.get_logger().error(f"Unknown flight mode: {rcin.mode}")

        if rcin.e_stop:
            self._estop.setText("ON")
        else:
            self._estop.setText("OFF")

        if rcin.gpsw:
            self._gpsw.setText("ON")
        else:
            self._gpsw.setText("OFF")
