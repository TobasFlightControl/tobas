from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import math
import rospy
from overrides import override
from PyQt5.QtCore import Qt, QPoint, QTimer
from PyQt5.QtGui import QPainter, QPaintEvent, QPolygon, QPen

from tobas_std_tools_py.math import wrap, ceil, floor
from tobas_kdl_msgs.msg import EulerStamped
from tobas_rqt_tools.widgets import Widget
from tobas_tools_py.drone import Drone

from ....common import PAINT_REFRESH_PERIOD
from .base_section import BaseControlSystemSectionWidget


class OrientationViewerWidget(BaseControlSystemSectionWidget):
    LABEL = "Orientation"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        self._orientation_viewer = _OrientationViewerWidget(main, drone)
        self._rows.addWidget(self._orientation_viewer)

    @override
    def define_connections(self) -> None:
        pass

    @override
    def update_internal_data_structures(self) -> None:
        self._orientation_viewer.update_internal_data_structures()


class _OrientationViewerWidget(Widget):

    W = 640
    H = 640
    LINE_WIDTH = 3  # ゲージ線の幅
    SCALE_INTERVAL = 10  # [deg]
    ROLL_RADIUS = 200  # ロール円の半径
    ROLL_TICK_LENGTH = 10
    PITCH_HEIGHT_RANGE = math.radians(120)
    PITCH_VISUAL_RANGE = 25  # [deg] 描画するピッチ角の範囲
    PITCH_LINE_LENGTH = 100
    YAW_WIDTH_RANGE = math.radians(120)
    YAW_LINE_Y = 60
    YAW_TICK_LENGTH = 10

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
        self._y_intercept = self.height() / 2

        self._euler_sub = None

        self._timer = QTimer(self)
        self._timer.timeout.connect(self.update)

    def update_internal_data_structures(self) -> None:
        self._reset()

        if self._euler_sub is not None:
            self._euler_sub.unregister()
        self._euler_sub = rospy.Subscriber(
            f"{self._drone.drone_name}/euler", EulerStamped, self._euler_cb, queue_size=1
        )

        self._timer.start(PAINT_REFRESH_PERIOD)

    @override
    def paintEvent(self, _: QPaintEvent) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        self._draw_ground(painter)
        self._draw_sky(painter)
        self._draw_roll(painter)
        self._draw_pitch(painter)
        self._draw_yaw(painter)

        painter.end()

    def _reset(self) -> None:
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._slope = 0.0
        self._y_intercept = self.height() / 2

    def _draw_ground(self, painter: QPainter) -> None:
        painter.fillRect(self.rect(), Qt.green)

    def _draw_sky(self, painter: QPainter) -> None:
        """空に含まれる領域を塗りつぶす． (memo: 2-59)"""
        tan_roll = math.tan(self._roll)
        tan_roll_sign = 1 if tan_roll >= 0 else -1
        tan_roll += tan_roll_sign * 1e-6  # tan(roll)が0になるのを防ぐ
        pitch_rate = 2 * self._pitch / self.PITCH_HEIGHT_RANGE

        OO = QPoint(0, 0)
        WO = QPoint(self.width(), 0)
        OH = QPoint(0, self.height())
        WH = QPoint(self.width(), self.height())
        XO = QPoint(int(self.width() + (1 - pitch_rate) * self.height() / tan_roll) // 2, 0)
        XH = QPoint(int(self.width() - (1 + pitch_rate) * self.height() / tan_roll) // 2, self.height())
        OY = QPoint(0, int(self.height() * (1 - pitch_rate) + self.width() * tan_roll) // 2)
        WY = QPoint(self.width(), int(self.height() * (1 - pitch_rate) - self.width() * tan_roll) // 2)

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
            points = [OO, WO, WH, OH]
        else:
            rospy.logerr("Impossible ground-sky pattern.")
            return

        polygon = QPolygon(points)
        painter.setBrush(Qt.blue)
        painter.drawPolygon(polygon)

        # リセット
        self._reset_painter(painter)

    def _is_sky(self, p: QPoint) -> bool:
        """カメラの枠内の点が空に含まれるかどうかを判定する．"""
        left = p.y()
        right = self._slope * p.x() + self._y_intercept

        # ロール角で場合分け．ロール角が90度を超えている場合は天地が逆転している．
        if abs(self._roll) < math.pi / 2:
            return left < right
        else:
            return left > right

    def _draw_roll(self, painter: QPainter) -> None:
        # 機体から見た円の中心に移動
        painter.translate(self.width() / 2, self.height() / 2)
        painter.rotate(-math.degrees(self._roll))

        # 円を描画
        painter.setPen(QPen(Qt.white, self.LINE_WIDTH))
        painter.drawEllipse(QPoint(0, 0), self.ROLL_RADIUS, self.ROLL_RADIUS)

        # 各値を描画
        outer_radius = self.ROLL_RADIUS + self.ROLL_TICK_LENGTH
        text_radius = outer_radius + 20
        for deg in range(0, 360, self.SCALE_INTERVAL):
            # 目盛りを描画
            painter.drawLine(0, -self.ROLL_RADIUS, 0, -outer_radius)

            # 数字を描画
            painter.drawText(-10, -text_radius, f"{wrap(deg, 180)}°")

            # 目盛りの間隔だけ進める
            painter.rotate(self.SCALE_INTERVAL)

        # 現在の位置に目印を描く
        self._reset_painter(painter)
        painter.translate(self.width() / 2, self.height() / 2)
        painter.setPen(QPen(Qt.red, self.LINE_WIDTH))
        painter.drawLine(0, -self.ROLL_RADIUS - self.ROLL_TICK_LENGTH * 2, 0, -self.ROLL_RADIUS)

        # リセット
        self._reset_painter(painter)

    def _draw_pitch(self, painter: QPainter) -> None:
        # 機体から見た中心位置に移動
        painter.translate(self.width() / 2, self.height() / 2)
        painter.rotate(-math.degrees(self._roll))

        # 描画する値の範囲を決める
        pitch_deg = math.degrees(self._pitch)
        pitch_min = floor(pitch_deg - self.PITCH_VISUAL_RANGE, self.SCALE_INTERVAL)
        pitch_max = ceil(pitch_deg + self.PITCH_VISUAL_RANGE, self.SCALE_INTERVAL)

        # 初期位置に移動
        painter.translate(0, self._pitch2height(math.radians(pitch_min - pitch_deg)))

        # 各値を描画
        line_half = self.PITCH_LINE_LENGTH // 2
        text_x = -line_half - 30
        text_y = 5
        y_interval = self._pitch2height(math.radians(self.SCALE_INTERVAL))
        painter.setPen(QPen(Qt.white, self.LINE_WIDTH))
        for deg in range(pitch_min, pitch_max + 1, self.SCALE_INTERVAL):
            # 目盛りを描画
            painter.drawLine(-line_half, 0, line_half, 0)

            # 数字を描画
            painter.drawText(text_x, text_y, f"{wrap(deg, 180)}°")

            # 目盛りの間隔だけ進める
            painter.translate(0, y_interval)

        # 現在の位置に目印を描く
        self._reset_painter(painter)
        painter.translate(self.width() / 2, self.height() / 2)
        painter.rotate(-math.degrees(self._roll))
        painter.setPen(QPen(Qt.red, self.LINE_WIDTH))
        painter.drawLine(-line_half, 0, line_half, 0)

        # リセット
        self._reset_painter(painter)

    def _draw_yaw(self, painter: QPainter) -> None:
        # 中心位置に移動
        beta = self.YAW_WIDTH_RANGE / 2  # [rad]
        painter.translate(self._yaw2width(beta), self.YAW_LINE_Y)

        # 数直線を描画
        painter.setPen(QPen(Qt.white, self.LINE_WIDTH))
        painter.drawLine(-self.width() / 2, 0, self.width() / 2, 0)

        # 描画する値の範囲を決める
        yaw_deg = math.degrees(self._yaw)
        yaw_min = floor(math.degrees(self._yaw - beta), self.SCALE_INTERVAL)
        yaw_max = ceil(math.degrees(self._yaw + beta), self.SCALE_INTERVAL)

        # 初期位置に移動
        painter.translate(self._yaw2width(math.radians(yaw_deg - yaw_min)), 0)

        # 各値を描画
        text_x = -10
        text_y = -self.YAW_TICK_LENGTH - 20
        x_interval = self._yaw2width(math.radians(self.SCALE_INTERVAL))
        for deg in range(yaw_min, yaw_max + 1, self.SCALE_INTERVAL):
            # 目盛りを描画
            painter.drawLine(0, 0, 0, -self.YAW_TICK_LENGTH)

            # 数字を描画
            painter.drawText(text_x, text_y, f"{wrap(deg, 180)}°")

            # 目盛りの間隔だけ進める
            painter.translate(-x_interval, 0)

        # 現在の位置に目印を描く
        self._reset_painter(painter)
        painter.translate(self._yaw2width(beta), self.YAW_LINE_Y)
        painter.setPen(QPen(Qt.red, self.LINE_WIDTH))
        painter.drawLine(0, 0, 0, -self.YAW_TICK_LENGTH * 2)

        # リセット
        self._reset_painter(painter)

    def _pitch2height(self, pitch: float) -> float:
        """ピッチ角 [rad] をウィンドウ高さに変換する．"""
        return self.height() * pitch / self.PITCH_HEIGHT_RANGE

    def _yaw2width(self, yaw: float) -> float:
        """ヨー角 [rad] をウィンドウ幅に変換する．"""
        return self.width() * yaw / self.YAW_WIDTH_RANGE

    @staticmethod
    def _reset_painter(painter: QPainter) -> None:
        painter.setPen(Qt.black)
        painter.setBrush(Qt.NoBrush)
        painter.resetTransform()

    def _euler_cb(self, euler: EulerStamped) -> None:
        euler_ = euler.euler

        if math.isnan(euler_.roll) or math.isnan(euler_.pitch) or math.isnan(euler_.yaw):
            rospy.logerr("NaN detected in euler angles.")
            self._reset()
            return

        self._roll = euler_.roll
        self._pitch = euler_.pitch
        self._yaw = euler_.yaw

        tan_roll = math.tan(euler_.roll)
        alpha = self.PITCH_HEIGHT_RANGE / 2
        self._slope = -tan_roll
        self._y_intercept = (self.width() * tan_roll + self.height() * (1 - euler_.pitch / alpha)) / 2
