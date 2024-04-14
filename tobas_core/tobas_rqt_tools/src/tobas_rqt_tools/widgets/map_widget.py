import os.path as osp
from overrides import override
from PyQt5.QtCore import Qt, QAbstractListModel, QModelIndex, QVariant, QUrl, QDateTime
from PyQt5.QtQuickWidgets import QQuickWidget
from PyQt5.QtPositioning import QGeoCoordinate


class MarkerModel(QAbstractListModel):
    PositionRole, SourceRole = range(Qt.UserRole, Qt.UserRole + 2)

    def __init__(self) -> None:
        super().__init__()
        self._markers = []

    @override
    def rowCount(self, parent: QModelIndex = QModelIndex()):
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

    def clear(self) -> None:
        self._markers.clear()

    def append_marker(self, marker: dict) -> None:
        self.beginInsertRows(QModelIndex(), self.rowCount(), self.rowCount())
        self._markers.append(marker)
        self.endInsertRows()


class MapWidget(QQuickWidget):
    """
    Open Street Mapを埋め込んだウィジェット．
    cf. https://stackoverflow.com/questions/36141170/creating-and-adding-mapquickitem-to-map-in-pyqt
    """

    MIN_INTERVAL = 100  # [ms]

    DEFAULT_LATITUDE = 35.6580992222
    DEFAULT_LONGITUDE = 139.7413574722
    DEFAULT_ZOOM_LEVEL = 10
    DEFAULT_ARROW_ROTATION = 0.0

    def __init__(self) -> None:
        super().__init__(resizeMode=QQuickWidget.SizeRootObjectToView)  # リサイズモードの指定が必須

        self._marker = MarkerModel()
        self.rootContext().setContextProperty("markermodel", self._marker)

        # QMLをセット
        qml_path = osp.join(osp.dirname(__file__), "qml/Map.qml")
        self.setSource(QUrl.fromLocalFile(qml_path))

        # 更新周波数が大きすぎるとバグるため，インターバルを設ける．
        t0 = QDateTime.fromSecsSinceEpoch(0)
        self._last_set_center = t0
        self._last_set_zoom_level = t0
        self._last_set_arrow_rotation = t0

    def clear(self) -> None:
        self._marker.clear()

    def append_marker(self, latitude: float, longitude: float, color: str = "red") -> None:
        assert -90 <= latitude <= 90
        assert -180 <= longitude <= 180

        coord = QGeoCoordinate(latitude, longitude)
        source = QUrl(f"http://maps.gstatic.com/mapfiles/ridefinder-images/mm_20_{color}.png")
        self._marker.append_marker({"position": coord, "source": source})

    def set_center(self, latitude: float, longitude: float) -> None:
        assert -90 <= latitude <= 90
        assert -180 <= longitude <= 180

        cur_time = QDateTime.currentDateTimeUtc()
        elapsed_time = self._last_set_center.msecsTo(cur_time)
        if elapsed_time < self.MIN_INTERVAL:
            return

        self.rootObject().setCenter.emit(latitude, longitude)
        self._last_set_center = cur_time

    def set_zoom_level(self, level: int) -> None:
        assert 0 <= level <= 20

        cur_time = QDateTime.currentDateTimeUtc()
        elapsed_time = self._last_set_zoom_level.msecsTo(cur_time)
        if elapsed_time < self.MIN_INTERVAL:
            return

        self.rootObject().setZoomLevel.emit(level)
        self._last_set_zoom_level = cur_time

    def set_arrow_rotation(self, angle: float) -> None:

        cur_time = QDateTime.currentDateTimeUtc()
        elapsed_time = self._last_set_arrow_rotation.msecsTo(cur_time)
        if elapsed_time < self.MIN_INTERVAL:
            return

        self.rootObject().setArrowRotation.emit(angle)
        self._last_set_arrow_rotation = cur_time
