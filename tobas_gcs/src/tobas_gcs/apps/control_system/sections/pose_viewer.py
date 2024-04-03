from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import math
import rospy
from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_kdl_msgs.msg import Euler
from tobas_tools_py.drone import Drone

from ....common import PAINT_REFRESH_DURATION
from .base_section import BaseControlSystemSectionWidget


class PoseViewerWidget(BaseControlSystemSectionWidget):
    LABEL = "Pose"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        cols = QHBoxLayout()
        self._rows.addLayout(cols)

        self._position_viewer = PositionViewerWidget(main, drone)
        cols.addWidget(self._position_viewer)

        self._altitude_viewer = AltitudeViewerWidget(main, drone)
        cols.addWidget(self._altitude_viewer)

        self._orientation_viewer = OrientationViewerWidget(main, drone)
        cols.addWidget(self._orientation_viewer)

        cols.addStretch()

    @override
    def define_connections(self) -> None:
        self._position_viewer.define_connections()
        self._altitude_viewer.define_connections()
        self._orientation_viewer.define_connections()

    @override
    def update_internal_data_structures(self) -> None:
        self._position_viewer.update_internal_data_structures()
        self._altitude_viewer.update_internal_data_structures()
        self._orientation_viewer.update_internal_data_structures()


class PositionViewerWidget(QWidget):

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__()
        self._main = main
        self._drone = drone

        # TODO

    def define_connections(self) -> None:
        pass

    def update_internal_data_structures(self) -> None:
        pass


class AltitudeViewerWidget(QWidget):

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__()
        self._main = main
        self._drone = drone

        # TODO

    def define_connections(self) -> None:
        pass

    def update_internal_data_structures(self) -> None:
        pass


class OrientationViewerWidget(QWidget):

    W = 300
    H = 300
    ALPHA = math.pi / 4  # ピッチ角の最大値
    EPS = 1e-6

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__()
        self._main = main
        self._drone = drone

        self.setFixedSize(self.W, self.H)

        # 現在のオイラー角
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0

        # 機体から見た地平線の方程式
        self._slope = 0.0
        self._y_intercept = self.H / 2

        self._euler_sub = None

        self._timer = QTimer(self)
        self._timer.timeout.connect(self.update)

    def define_connections(self) -> None:
        pass

    def update_internal_data_structures(self) -> None:
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._slope = 0.0
        self._y_intercept = self.H / 2

        if self._euler_sub is not None:
            self._euler_sub.unregister()
        self._euler_sub = rospy.Subscriber(f"/{self._drone.drone_name}/euler", Euler, self._euler_cb, queue_size=1)

        self._timer.start(PAINT_REFRESH_DURATION)

    @override
    def paintEvent(self, _: QPaintEvent) -> None:
        painter = QPainter(self)

        self._draw_ground(painter)
        self._draw_sky(painter)

        painter.end()

    def _draw_ground(self, painter: QPainter) -> None:
        painter.fillRect(self.rect(), Qt.green)

    def _draw_sky(self, painter: QPainter) -> None:
        """空に含まれる領域を塗りつぶす． (memo: 2-59)"""
        tan_roll = math.tan(self._roll)
        pitch_rate = self._pitch / self.ALPHA

        OO = QPoint(0, 0)
        WO = QPoint(self.W, 0)
        OH = QPoint(0, self.H)
        WH = QPoint(self.W, self.H)
        XO = QPoint((self.W + (1 - pitch_rate) * self.H / (tan_roll + self.EPS)) / 2, 0)
        XH = QPoint((self.W - (1 + pitch_rate) * self.H / (tan_roll + self.EPS)) / 2, self.H)
        OY = QPoint(0, (self.H * (1 - pitch_rate) + self.W * tan_roll) / 2)
        WY = QPoint(self.W, (self.H * (1 - pitch_rate) - self.W * tan_roll) / 2)

        OO_sky = self._is_sky(OO)
        WO_sky = self._is_sky(WO)
        OH_sky = self._is_sky(OH)
        WH_sky = self._is_sky(WH)

        if not OO_sky and not WO_sky and not OH_sky and not WH_sky:  # 0000
            points = []
        elif not OO_sky and not WO_sky and not OH_sky and WH_sky:  # 0001
            points = [WH, XH, WY]
        elif not OO_sky and not WO_sky and OH_sky and not WH_sky:  # 0010
            points = [OH, XH, OY]
        elif not OO_sky and not WO_sky and OH_sky and WH_sky:  # 0011
            points = [OH, WH, WY, OY]
        elif not OO_sky and WO_sky and not OH_sky and not WH_sky:  # 0100
            points = [WO, XO, WY]
        elif not OO_sky and WO_sky and not OH_sky and WH_sky:  # 0101
            points = [WO, WH, XH, XO]
        elif not OO_sky and WO_sky and OH_sky and WH_sky:  # 0111
            points = [WO, WH, OH, OY, XO]
        elif OO_sky and not WO_sky and not OH_sky and not WH_sky:  # 1000
            points = [OO, XO, OY]
        elif OO_sky and not WO_sky and OH_sky and not WH_sky:  # 1010
            points = [OO, OH, XH, XO]
        elif OO_sky and not WO_sky and OH_sky and WH_sky:  # 1011
            points = [OO, OH, WH, WY, XO]
        elif OO_sky and WO_sky and not OH_sky and not WH_sky:  # 1100
            points = [OO, WO, WY, OY]
        elif OO_sky and WO_sky and not OH_sky and WH_sky:  # 1101
            points = [OO, WO, WH, XH, OY]
        elif OO_sky and WO_sky and OH_sky and not WH_sky:  # 1110
            points = [WO, OO, OH, XH, WY]
        elif OO_sky and WO_sky and OH_sky and WH_sky:  # 1111
            points[OO, WO, WH, OH]
        else:
            raise RuntimeError("Impossible pattern.")

        polygon = QPolygon(points)
        painter.setBrush(Qt.blue)
        painter.drawPolygon(polygon)

    def _is_sky(self, p: QPoint) -> bool:
        """カメラの枠内の点が空に含まれるかどうかを判定する．"""
        left = p.y()
        right = self._slope * p.x() + self._y_intercept

        # ロール角で場合分け．ロール角が90度を超えている場合は天地が逆転している．
        if -math.pi / 2 < self._roll < math.pi / 2:
            return left < right
        else:
            return left > right

    def _euler_cb(self, euler: Euler) -> None:
        self._roll = euler.roll
        self._pitch = euler.pitch
        self._yaw = euler.yaw

        tan_roll = math.tan(euler.roll)
        self._slope = -tan_roll
        self._y_intercept = (self.W * tan_roll + self.H * (1 - euler.pitch / self.ALPHA)) / 2
