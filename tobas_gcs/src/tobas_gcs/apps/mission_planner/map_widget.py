import os.path as osp
from overrides import override
from typing import Tuple, List, Tuple, Dict, Type
from PyQt5.QtCore import Qt, QObject, QAbstractListModel, QModelIndex, QVariant, QUrl
from PyQt5.QtQuickWidgets import QQuickWidget
from PyQt5.QtPositioning import QGeoCoordinate


class AbstractListModel(QAbstractListModel):
    FIELDS: List[Tuple[str, Type]] = []

    def __init__(self) -> None:
        super().__init__()
        self._list: List[Tuple] = []

    @override
    def rowCount(self, _: QModelIndex = QModelIndex()) -> int:
        return len(self._list)

    @override
    def data(self, index: QModelIndex, role: Qt.ItemDataRole) -> QVariant:
        return self._list[index.row()][role - Qt.UserRole]

    @override
    def roleNames(self) -> Dict[Qt.ItemDataRole, bytes]:
        res = dict()
        for i, (name, _) in enumerate(self.FIELDS):
            res[Qt.UserRole + i] = name.encode()
        return res

    def add(self, *args) -> None:
        assert len(args) == len(self.FIELDS)
        for arg, (name, type_) in zip(args, self.FIELDS):
            assert isinstance(arg, type_), f"{arg}, {name}, {type_}"

        self.beginInsertRows(QModelIndex(), self.rowCount(), self.rowCount())
        self._list.append(args)
        self.endInsertRows()

    def clear(self) -> None:
        self.beginRemoveRows(QModelIndex(), 0, self.rowCount() - 1)
        self._list.clear()
        self.endRemoveRows()


class MarkerModel(AbstractListModel):
    FIELDS = [("position", QGeoCoordinate), ("source", QUrl)]


class LineModel(AbstractListModel):
    FIELDS = [("latitude_1", float), ("longitude_1", float), ("latitude_2", float), ("longitude_2", float)]


class MapWidget(QQuickWidget):
    """
    Open Street Mapを埋め込んだウィジェット．
    cf. https://stackoverflow.com/questions/36141170/creating-and-adding-mapquickitem-to-map-in-pyqt
    """

    def __init__(self) -> None:
        super().__init__(resizeMode=QQuickWidget.SizeRootObjectToView)  # リサイズモードの指定が必須

        self._marker = MarkerModel()
        self.rootContext().setContextProperty(MarkerModel.__name__, self._marker)

        self._line = LineModel()
        self.rootContext().setContextProperty(LineModel.__name__, self._line)

        # QMLをセット
        qml_path = osp.join(osp.dirname(__file__), "qml/Map.qml")
        self.setSource(QUrl.fromLocalFile(qml_path))

    def clear(self) -> None:
        self._marker.clear()
        self._line.clear()

    def add_marker(self, latitude: float, longitude: float, color: str = "red") -> None:
        assert -90 <= latitude <= 90
        assert -180 <= longitude <= 180

        coord = QGeoCoordinate(latitude, longitude)
        source = QUrl(f"http://maps.gstatic.com/mapfiles/ridefinder-images/mm_20_{color}.png")  # TODO: ローカルに保存
        self._marker.add(coord, source)

    def add_line(self, latitude_1: float, longitude_1: float, latitude_2: float, longitude_2: float) -> None:
        self._line.add(latitude_1, longitude_1, latitude_2, longitude_2)

    def get_center(self) -> Tuple[float, float]:
        map = self.rootObject().findChild(QObject, "map")
        center: QGeoCoordinate = map.property("center")
        return center.latitude(), center.longitude()

    def set_center(self, latitude: float, longitude: float) -> None:
        assert -90 <= latitude <= 90
        assert -180 <= longitude <= 180

        self.rootObject().setCenter.emit(latitude, longitude)
