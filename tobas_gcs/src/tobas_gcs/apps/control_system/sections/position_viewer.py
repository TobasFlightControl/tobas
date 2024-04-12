from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....gcs import GroundControlStationWidget

import os.path as osp
import math
import rospy
from overrides import override
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtQuickWidgets import QQuickWidget
from PyQt5.QtPositioning import QGeoCoordinate

from tobas_rqt_tools.widgets import FramedLabel
from tobas_tools_py.drone import Drone
from tobas_msgs.msg import Gps

from .base_section import BaseControlSystemSectionWidget


class PositionViewerWidget(BaseControlSystemSectionWidget):
    LABEL = "Position"

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        self._map = MapWidget()
        self._rows.addWidget(self._map)

        self._latitude = LabelTextWidget("Latitude")
        self._longitude = LabelTextWidget("Longitude")
        self._altitude = LabelTextWidget("Altitude")
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

        self._gps_sub = None

    @override
    def define_connections(self) -> None:
        pass

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
        self._gps_sub = rospy.Subscriber(f"/{self._drone.drone_name}/gps", Gps, self._gps_cb, queue_size=1)

    def _gps_cb(self, gps: Gps) -> None:
        if gps.fix_type != Gps.FIX_3D:
            return

        self._latitude.set_text(f"{gps.latitude:.9f} deg")
        self._longitude.set_text(f"{gps.longitude:.9f} deg")
        self._altitude.set_text(f"{gps.altitude:.3f} m")
        self._x_stddev.set_text(f"{math.sqrt(gps.position_covariance[0]):.3f} m")
        self._y_stddev.set_text(f"{math.sqrt(gps.position_covariance[4]):.3f} m")
        self._z_stddev.set_text(f"{math.sqrt(gps.position_covariance[8]):.3f} m")


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


class MarkerModel(QAbstractListModel):
    PositionRole, SourceRole = range(Qt.UserRole, Qt.UserRole + 2)

    def __init__(self):
        super().__init__()
        self._markers = []

    @override
    def rowCount(self, parent=QModelIndex()):
        return len(self._markers)

    @override
    def data(self, index: QModelIndex, role: Qt.ItemDataRole = Qt.DisplayRole):
        if 0 <= index.row() < self.rowCount():
            if role == MarkerModel.PositionRole:
                return self._markers[index.row()]["position"]
            elif role == MarkerModel.SourceRole:
                return self._markers[index.row()]["source"]
        return QVariant()

    @override
    def roleNames(self):
        return {MarkerModel.PositionRole: b"position_marker", MarkerModel.SourceRole: b"source_marker"}

    def append_marker(self, marker):
        self.beginInsertRows(QModelIndex(), self.rowCount(), self.rowCount())
        self._markers.append(marker)
        self.endInsertRows()


class MapWidget(QQuickWidget):
    """
    マップウィジェット．
    cf. https://stackoverflow.com/questions/36141170/creating-and-adding-mapquickitem-to-map-in-pyqt
    """

    WIDTH = 640
    HEIGHT = 480

    def __init__(self):
        super().__init__()

        # リサイズモードを指定した上でのサイズ固定が必須
        self.setResizeMode(QQuickWidget.SizeRootObjectToView)
        self.setFixedSize(self.WIDTH, self.HEIGHT)

        self._marker = MarkerModel()
        self.rootContext().setContextProperty("markermodel", self._marker)

        # QMLをセット
        qml_path = osp.join(osp.dirname(__file__), "Map.qml")
        self.setSource(QUrl.fromLocalFile(qml_path))

    def append_marker(self, latitude: float, longitude: float, color: str = "red") -> None:
        coord = QGeoCoordinate(latitude, longitude)
        source = QUrl(f"http://maps.gstatic.com/mapfiles/ridefinder-images/mm_20_{color}.png")
        self._marker.append_marker({"position": coord, "source": source})
