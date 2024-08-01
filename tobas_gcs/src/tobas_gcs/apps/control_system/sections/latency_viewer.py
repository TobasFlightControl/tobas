from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import rclpy
from rclpy.duration import Duration
import pyqtgraph as pg
from overrides import override
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QWheelEvent

from tobas_rclpy.timestamped_buffer import TimestampedBuffer
from tobas_tools_py.constants import Topic
from tobas_tools_py.drone import Drone
from tobas_msgs.msg import Latency

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

        self._buffer = TimestampedBuffer[Duration](Duration(self.EXPIRY_DURATION))
        self._latency_sub = None

    @override
    def update_internal_data_structures(self) -> None:
        self._pw.plotItem.clear()
        self._buffer.clear()

        if self._latency_sub is not None:
            self._latency_sub.unregister()
        self._latency_sub = rclpy.Subscriber(
            f"{self._drone.name}/{Topic.Throttled.LATENCY}",
            Latency,
            self._latency_cb,
            queue_size=1,
        )

    def _latency_cb(self, latency: Latency) -> None:
        self._buffer.add(latency.header.stamp, latency.data)

        # PlotWidgetが定義されたスレッドとROSコールバックのスレッドが異なるため，ここから直接PlotWidgetのメソッドを呼ぶことはできない．
        # そのため，一度QTimerを介してQtのスレッドからPlotWidgetのメソッドを呼ぶようにする．
        QTimer.singleShot(0, self._update_plot)

    def _update_plot(self) -> None:
        stamps = []
        latencies = []
        for stamp, latency in self._buffer:
            stamps.append(stamp.to_sec())
            latencies.append(latency.to_nsec() * 1e-6)

        pi = self._pw.plotItem
        pi.clear()
        pi.addItem(pg.PlotCurveItem(x=stamps, y=latencies, pen=pg.mkPen(color="k"), antialias=True))
        pi.setLabels(bottom="Time [s]", left="Latency [ms]")
        pi.vb.setLimits(yMin=0)
