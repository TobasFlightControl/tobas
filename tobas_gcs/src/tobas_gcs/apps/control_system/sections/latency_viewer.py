from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import rospy
import pyqtgraph as pg
from overrides import override
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QWheelEvent

from tobas_rospy.timestamped_buffer import TimestampedBuffer
from tobas_tools_py.drone import Drone
from tobas_msgs.msg import Latency

from ....common import PAINT_REFRESH_PERIOD
from .base_section import BaseControlSystemSectionWidget


class PlotWidget(pg.PlotWidget):
    def wheelEvent(self, e: QWheelEvent) -> None:
        # マウスホイールイベントを無効化
        # なぜか@overrideは付けられない
        e.ignore()


class LatencyViewerWidget(BaseControlSystemSectionWidget):
    LABEL = "Control Latency"

    PLOT_HEIGHT = 300
    EXPIRY_DURATION = 10  # [s]

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        self._pw = PlotWidget()
        self._pw.setFixedHeight(self.PLOT_HEIGHT)
        self._pw.setBackground("w")
        self._rows.addWidget(self._pw)

        self._buffer = TimestampedBuffer(rospy.Duration(self.EXPIRY_DURATION))
        self._latency_sub = None

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._timer_cb)

    @override
    def update_internal_data_structures(self) -> None:
        self._pw.plotItem.clear()
        self._buffer.clear()

        if self._latency_sub is not None:
            self._latency_sub.unregister()
        self._latency_sub = rospy.Subscriber(
            f"{self._drone.drone_name}/latency", Latency, self._latency_cb, queue_size=1
        )

        self._timer.start(PAINT_REFRESH_PERIOD)

    def _latency_cb(self, latency: Latency) -> None:
        self._buffer.add(latency.header.stamp, latency.data)

    def _timer_cb(self) -> None:
        stamps = []
        latencies = []
        for stamp, latency in self._buffer:
            stamps.append(stamp.to_sec())
            latencies.append(latency * 1e3)

        pi = self._pw.plotItem
        pi.clear()
        pi.addItem(pg.PlotCurveItem(x=stamps, y=latencies, pen=pg.mkPen(color="k"), antialias=True))
        pi.setLabels(bottom="Time [s]", left="Latency [ms]")
        pi.vb.setLimits(yMin=0)
