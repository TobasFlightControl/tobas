from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .....gcs import GroundControlStationWidget

import os.path as osp
import math
import rospy
from overrides import override
from PyQt5.QtCore import QObject, pyqtSignal, pyqtProperty, QStandardPaths, QDateTime, QUrl
from PyQt5.QtWidgets import QWidget, QLabel, QHBoxLayout, QGridLayout, QSizePolicy
from PyQt5.QtQuickWidgets import QQuickWidget

from tobas_rqt_tools.widgets import FramedLabel
from tobas_tools_py.constants import Topic
from tobas_tools_py.drone import Drone
from tobas_kdl_msgs.msg import EulerStamped
from tobas_msgs.msg import Gps

from ..base_section import BaseControlSystemSectionWidget


class LabelTextWidget(QWidget):

    TEXT_WIDTH = 150

    def __init__(self, label: str) -> None:
        super().__init__()

        cols = QHBoxLayout()
        self.setLayout(cols)

        cols.addWidget(QLabel(label))

        self._text = FramedLabel()
        self._text.setFixedWidth(self.TEXT_WIDTH)
        cols.addWidget(self._text)

    def set_text(self, text: str) -> None:
        self._text.setText(text)

    def clear(self) -> None:
        self._text.clear()


class SystemInfo(QObject):
    """QMLのコンストラクタ引数．"""

    _ = pyqtSignal()

    def __init__(self, parent=None):
        super(SystemInfo, self).__init__(parent)
        self._home_dir = QStandardPaths.writableLocation(QStandardPaths.HomeLocation)

    @pyqtProperty(str, notify=_)
    def homeDirectory(self):
        return self._home_dir


class MapWidget(QQuickWidget):
    """
    Open Street Mapを埋め込んだウィジェット．
    cf. https://stackoverflow.com/questions/36141170/creating-and-adding-mapquickitem-to-map-in-pyqt
    """

    MIN_INTERVAL = 100  # [ms]

    def __init__(self) -> None:
        super().__init__(resizeMode=QQuickWidget.SizeRootObjectToView)  # リサイズモードの指定が必須

        # コンストラクタ引数を設定
        system_info = SystemInfo()
        self.rootContext().setContextProperty(SystemInfo.__name__, system_info)

        # QMLを読み込む
        qml_path = osp.join(osp.dirname(__file__), "Map.qml")
        self.setSource(QUrl.fromLocalFile(qml_path))

        # 更新周波数が大きすぎるとバグるため，インターバルを設ける．
        t0 = QDateTime.fromSecsSinceEpoch(0)
        self._last_set_center = t0
        self._last_set_arrow_rotation = t0

    def set_center(self, latitude: float, longitude: float) -> None:
        assert -90 <= latitude <= 90
        assert -180 <= longitude <= 180

        cur_time = QDateTime.currentDateTimeUtc()
        elapsed_time = self._last_set_center.msecsTo(cur_time)
        if elapsed_time < self.MIN_INTERVAL:
            return

        self.rootObject().setCenter.emit(latitude, longitude)
        self._last_set_center = cur_time

    def set_arrow_rotation(self, angle: float) -> None:

        cur_time = QDateTime.currentDateTimeUtc()
        elapsed_time = self._last_set_arrow_rotation.msecsTo(cur_time)
        if elapsed_time < self.MIN_INTERVAL:
            return

        self.rootObject().setArrowRotation.emit(angle)
        self._last_set_arrow_rotation = cur_time


class PositionViewerWidget(BaseControlSystemSectionWidget):
    LABEL = "Position"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        self._map = MapWidget()
        self._map.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self._rows.addWidget(self._map)

        self._latitude = LabelTextWidget("Latitude")
        self._longitude = LabelTextWidget("Longitude")
        self._altitude = LabelTextWidget("Altitude (MSL)")
        self._x_stddev = LabelTextWidget("X std. dev")
        self._y_stddev = LabelTextWidget("Y std. dev")
        self._z_stddev = LabelTextWidget("Z std. dev")

        grid = QGridLayout()
        self._rows.addLayout(grid)
        grid.addWidget(self._latitude, 0, 0)
        grid.addWidget(self._longitude, 0, 1)
        grid.addWidget(self._altitude, 0, 2)
        grid.addWidget(self._x_stddev, 1, 0)
        grid.addWidget(self._y_stddev, 1, 1)
        grid.addWidget(self._z_stddev, 1, 2)
        grid.setColumnStretch(3, 1)

        self._gps_sub = None
        self._euler_sub = None

    @override
    def update_internal_data_structures(self) -> None:
        self._latitude.clear()
        self._longitude.clear()
        self._altitude.clear()
        self._x_stddev.clear()
        self._y_stddev.clear()
        self._z_stddev.clear()

        if self._gps_sub is not None:
            self._gps_sub.unregister()
            self._euler_sub.unregister()
        self._gps_sub = rospy.Subscriber(f"{self._drone.drone_name}/{Topic.GNSS}", Gps, self._gps_cb, queue_size=1)
        self._euler_sub = rospy.Subscriber(
            f"{self._drone.drone_name}/{Topic.EULER}", EulerStamped, self._euler_cb, queue_size=1
        )

    def _gps_cb(self, gps: Gps) -> None:
        if gps.fix_type != Gps.FIX_3D:
            return

        self._map.set_center(gps.latitude, gps.longitude)

        self._latitude.set_text(f"{gps.latitude:.9f} deg")
        self._longitude.set_text(f"{gps.longitude:.9f} deg")
        self._altitude.set_text(f"{gps.altitude:.3f} m")
        self._x_stddev.set_text(f"{math.sqrt(gps.position_covariance[0]):.3f} m")
        self._y_stddev.set_text(f"{math.sqrt(gps.position_covariance[4]):.3f} m")
        self._z_stddev.set_text(f"{math.sqrt(gps.position_covariance[8]):.3f} m")

    def _euler_cb(self, euler: EulerStamped) -> None:
        yaw_deg = -math.degrees(euler.euler.yaw)
        self._map.set_arrow_rotation(yaw_deg)
