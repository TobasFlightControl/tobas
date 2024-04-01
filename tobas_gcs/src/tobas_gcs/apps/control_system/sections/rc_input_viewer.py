from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import rospy
from abc import abstractmethod
from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_std_tools_py.math import remap
from tobas_rqt_tools.widgets import FramedLabel
from tobas_rqt_tools.utils import place_center, create_fixed_height_hboxlayout
from tobas_tools_py.drone import Drone
from tobas_msgs.msg import RCInput, RCInputError

from .base_section import BaseControlSystemSectionWidget


class PositionBarWidget(QWidget):
    LINE_WIDTH = 3
    TEXT_PSIZE = 10

    def __init__(self, lower: int, upper: int, parent: QWidget = None) -> None:
        super().__init__(parent)

        self._lower = lower
        self._upper = upper

        self._value = None

    def get_value(self) -> int:
        return self._value

    def set_value(self, value: int) -> None:
        self._value = value
        self.update()

    def clear(self) -> None:
        self._value = None
        self.update()

    @abstractmethod
    def paintEvent(self, event: QPaintEvent) -> None:
        raise NotImplementedError()


class HPositionBarWidget(PositionBarWidget):
    @override
    def paintEvent(self, event: QPaintEvent) -> None:
        # QPainterはpaintEvent内でのみ定義できる
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # 背景を描画
        painter.fillRect(event.rect(), Qt.white)

        # 枠を描画
        painter.setPen(Qt.black)
        painter.drawRect(0, 0, self.width(), self.height())

        if self._value is not None:
            # バーの位置を計算
            value_pos = remap(self._value, self._lower, self._upper, 0, self.width())

            # 現在値の位置に赤色の線を描画
            painter.setPen(QPen(Qt.red, self.LINE_WIDTH))
            painter.drawLine(value_pos, 0, value_pos, self.height())

        # Painterを破棄 (適切に破棄しないとメモリリークが起きる)
        painter.end()


class VPositionBarWidget(PositionBarWidget):
    @override
    def paintEvent(self, event: QPaintEvent) -> None:
        # QPainterはpaintEvent内でのみ定義できる
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # 背景を描画
        painter.fillRect(event.rect(), Qt.white)

        # 枠を描画
        painter.setPen(Qt.black)
        painter.drawRect(0, 0, self.width(), self.height())

        if self._value is not None:
            # バーの位置を計算
            value_pos = remap(self._value, self._lower, self._upper, 0, self.height())

            # 現在値の位置に赤色の線を描画
            painter.setPen(QPen(Qt.red, self.LINE_WIDTH))
            painter.drawLine(0, value_pos, self.width(), value_pos)

        # Painterを破棄 (適切に破棄しないとメモリリークが起きる)
        painter.end()


class RCInputViewerWidget(BaseControlSystemSectionWidget):
    LABEL = "Radio Input"

    SCALAR = 10000
    RANGE_SIDE_SHORT = 30
    RANGE_SIDE_LONG = 300
    LABEL_WIDTH = 100
    LABEL_HEIGHT = 30

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        cols1 = QHBoxLayout()
        self._rows.addLayout(cols1)

        # Roll, Pitch, Yaw, Thrust
        cols2 = create_fixed_height_hboxlayout(self.RANGE_SIDE_LONG + 20, cols1)

        self._pitch_range = VPositionBarWidget(self.SCALAR, -self.SCALAR)
        self._pitch_range.setFixedSize(self.RANGE_SIDE_SHORT, self.RANGE_SIDE_LONG)
        cols2.addWidget(self._pitch_range)

        rows1 = QVBoxLayout()
        cols2.addLayout(rows1)

        self._roll_range = HPositionBarWidget(-self.SCALAR, self.SCALAR)
        self._roll_range.setFixedSize(self.RANGE_SIDE_LONG, self.RANGE_SIDE_SHORT)
        place_center(self._roll_range, rows1)
        place_center(QLabel(f"Roll"), rows1)

        rows1.addStretch()

        cols3 = QHBoxLayout()
        rows1.addLayout(cols3)

        pitch_label = QLabel(f"Pitch")
        pitch_label.setAlignment(Qt.AlignLeft)
        cols3.addWidget(pitch_label)

        thrust_label = QLabel(f"Thrust")
        thrust_label.setAlignment(Qt.AlignRight)
        cols3.addWidget(thrust_label)

        rows1.addStretch()

        place_center(QLabel(f"Yaw"), rows1)
        self._yaw_range = HPositionBarWidget(self.SCALAR, -self.SCALAR)
        self._yaw_range.setFixedSize(self.RANGE_SIDE_LONG, self.RANGE_SIDE_SHORT)
        place_center(self._yaw_range, rows1)

        self._thrust_range = VPositionBarWidget(self.SCALAR, 0)
        self._thrust_range.setFixedSize(self.RANGE_SIDE_SHORT, self.RANGE_SIDE_LONG)
        cols2.addWidget(self._thrust_range)

        cols1.addSpacing(30)

        bar_grid = QGridLayout()
        cols1.addLayout(bar_grid)

        # Mode
        bar_grid.addWidget(QLabel(f"Mode  :"), 0, 0)
        self._mode = FramedLabel(parent=self)  # QLineEditだと処理が重すぎるのか落ちてしまう
        self._mode.setFixedSize(self.LABEL_WIDTH, self.LABEL_HEIGHT)
        bar_grid.addWidget(self._mode, 0, 1)

        # E-Stop
        bar_grid.addWidget(QLabel(f"E-Stop:"), 1, 0)
        self._estop = FramedLabel(parent=self)
        self._estop.setFixedSize(self.LABEL_WIDTH, self.LABEL_HEIGHT)
        bar_grid.addWidget(self._estop, 1, 1)

        # GPSw
        bar_grid.addWidget(QLabel(f"GPSw  :"), 2, 0)
        self._gpsw = FramedLabel(parent=self)
        self._gpsw.setFixedSize(self.LABEL_WIDTH, self.LABEL_HEIGHT)
        bar_grid.addWidget(self._gpsw, 2, 1)

        cols1.addStretch()

        # Subscriber
        self._rcin_sub = None

    @override
    def define_connections(self) -> None:
        pass

    @override
    def update_internal_data_structures(self) -> None:
        self._clear()

        if self._rcin_sub is not None:
            self._rcin_sub.unregister()
        self._rcin_sub = rospy.Subscriber(f"/{self._drone.drone_name}/rc_input", RCInput, self._rcin_cb, queue_size=1)

    def _clear(self) -> None:
        self._roll_range.clear()
        self._pitch_range.clear()
        self._yaw_range.clear()
        self._thrust_range.clear()
        self._mode.clear()
        self._estop.clear()
        self._gpsw.clear()

    def _rcin_cb(self, rcin: RCInput) -> None:
        if rcin.error.error != RCInputError.E_NO_ERROR:
            return

        self._roll_range.set_value(rcin.roll * self.SCALAR)
        self._pitch_range.set_value(rcin.pitch * self.SCALAR)
        self._yaw_range.set_value(rcin.yaw * self.SCALAR)
        self._thrust_range.set_value(rcin.thrust * self.SCALAR)

        if rcin.mode == RCInput.MODE_PROGRAM:
            self._mode.setText("Program")
        elif rcin.mode == RCInput.MODE_STABILIZE:
            self._mode.setText("Stabilize")
        elif rcin.mode == RCInput.MODE_ACROBAT:
            self._mode.setText("Acrobat")
        else:
            rospy.logerr(f"Unknown flight mode: {rcin.mode}")

        if rcin.e_stop:
            self._estop.setText("ON")
        else:
            self._estop.setText("OFF")

        if rcin.gpsw:
            self._gpsw.setText("ON")
        else:
            self._gpsw.setText("OFF")
