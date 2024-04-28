from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

from overrides import override
from functools import partial
from typing import Tuple, List
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtPositioning import QGeoCoordinate

from tobas_rqt_tools.widgets import ListWidget
from tobas_tools_py.drone import Drone

from ..base import BaseAppWidget
from .map_widget import MapWidget
from .add_command_dialog import Commands, AddCommandDialog
from .command_properties import *


class MissionPlannerWidget(BaseAppWidget):
    NAME = "Mission Planner"

    BUTTON_WIDTH = 100
    BUTTON_HEIGHT = 40

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        rows = QVBoxLayout()
        self.setLayout(rows)

        self._map = MapWidget()
        self._map.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        rows.addWidget(self._map)

        button_cols = QHBoxLayout()
        rows.addLayout(button_cols)

        self._load_button = QPushButton("Load")
        self._load_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        button_cols.addWidget(self._load_button)

        self._save_button = QPushButton("Save")
        self._save_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        button_cols.addWidget(self._save_button)

        self._add_button = QPushButton("Add")
        self._add_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        button_cols.addWidget(self._add_button)

        self._clear_button = QPushButton("Clear")
        self._clear_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        button_cols.addWidget(self._clear_button)

        button_cols.addStretch()

        self._execute_button = QPushButton("Execute")
        self._execute_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        button_cols.addWidget(self._execute_button)

        self._cancel_button = QPushButton("Cancel")
        self._cancel_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        button_cols.addWidget(self._cancel_button)

        mission_cols = QHBoxLayout()
        rows.addLayout(mission_cols)

        self._command_list = ListWidget()
        self._command_list.setSelectionMode(QListWidget.SingleSelection)
        self._command_list.setDragDropMode(QListWidget.InternalMove)
        mission_cols.addWidget(self._command_list)

        self._properties = QStackedWidget()
        self._properties.setStyleSheet("QStackedWidget { border: 1px solid black; background-color: white; }")
        mission_cols.addWidget(self._properties)

        self._pairs: List[Tuple[QListWidgetItem, BasePropertyWidget]] = []

    @override
    def define_connections(self) -> None:
        self._load_button.clicked.connect(self._on_load_button_clicked)
        self._save_button.clicked.connect(self._on_save_button_clicked)
        self._add_button.clicked.connect(self._on_add_button_clicked)
        self._clear_button.clicked.connect(self._on_clear_button_clicked)
        self._execute_button.clicked.connect(self._on_execute_button_clicked)
        self._cancel_button.clicked.connect(self._on_cancel_button_clicked)

        self._command_list.itemClicked.connect(self._on_list_item_changed)
        self._command_list.item_moved.connect(self._on_list_item_changed)

        self._map.waypoint_moved.connect(self._on_waypoint_moved)

    @override
    def update_internal_data_structures(self) -> None:
        pass

    @pyqtSlot()
    def _on_load_button_clicked(self) -> None:
        pass  # TODO

    @pyqtSlot()
    def _on_save_button_clicked(self) -> None:
        pass  # TODO

    @pyqtSlot()
    def _on_add_button_clicked(self) -> None:
        dialog = AddCommandDialog(self)

        res = dialog.exec()
        if res != QDialog.Accepted:
            return

        command = dialog.selected_command()
        if command == Commands.WAYPOINT.value:
            prop = WaypointPropertyWidget()
            latitude, longitude = self._map.get_center()
            prop.latitude.setValue(latitude)
            prop.longitude.setValue(longitude)
        elif command == Commands.TAKEOFF.value:
            prop = TakeoffPropertyWidget()
        elif command == Commands.LAND.value:
            prop = LandPropertyWidget()
        elif command == Commands.RETURN_TO_HOME.value:
            prop = RTHPropertyWidget()
        else:
            raise RuntimeError(f"Unknown command: {command}")

        item = QListWidgetItem(command)
        self._command_list.addItem(item)

        self._properties.addWidget(prop)
        prop.value_changed.connect(self._on_property_value_changed)
        prop.delete_button_clicked.connect(partial(self._on_delete_button_clicked, target_item=item, target_prop=prop))

        self._pairs.append((item, prop))

        self._update_properties()
        self._update_map()

    @pyqtSlot()
    def _on_clear_button_clicked(self) -> None:
        pass  # TODO

    @pyqtSlot()
    def _on_execute_button_clicked(self) -> None:
        pass  # TODO

    @pyqtSlot()
    def _on_cancel_button_clicked(self) -> None:
        pass  # TODO

    @pyqtSlot()
    def _on_property_value_changed(self) -> None:
        self._update_map()

    @pyqtSlot()
    def _on_delete_button_clicked(self, target_item: QListWidgetItem, target_prop: BasePropertyWidget) -> None:
        self._command_list.remove(target_item)
        self._properties.removeWidget(target_prop)

        for item, prop in self._pairs:
            if item is target_item and prop is target_prop:
                self._pairs.remove((item, prop))
                break
        else:
            raise RuntimeError()

        self._update_properties()
        self._update_map()

    @pyqtSlot()
    def _on_list_item_changed(self):
        self._update_properties()
        self._update_map()

    @pyqtSlot(int, float, float)
    def _on_waypoint_moved(self, index: int, latitude: float, longitude: float) -> None:
        cur_idx = 0
        for item in self._command_list:
            command = item.text()
            if command == Commands.WAYPOINT.value:
                cur_idx += 1
            if cur_idx == index:
                prop: WaypointPropertyWidget = self._get_property(item)
                prop.latitude.setValue(latitude)
                prop.longitude.setValue(longitude)
                break
        else:
            raise RuntimeError(f"Index {index} is out of range.")

    def _update_properties(self) -> None:
        """選択されているリストアイテムに基づいてプロパティの表示を更新．"""
        if self._command_list.count() == 0:
            return

        # 選択されているアイテムを取得
        selected_items = self._command_list.selectedItems()
        if len(selected_items) > 0:
            selected_item = selected_items[0]
        else:  # 何も選択されていなければ強制的に最初の要素を選択する
            self._command_list.setCurrentRow(0)
            selected_item = self._command_list.item(0)

        for item, prop in self._pairs:
            if item is selected_item:
                self._properties.setCurrentWidget(prop)
                return
        else:
            raise RuntimeError()

    def _update_map(self) -> None:
        """現在のコマンドに基づいてマップ上のオブジェクトを描き直す．"""
        self._map.clear()

        index = 1
        last_coord: Tuple[float, float] = None

        for item in self._command_list:
            command = item.text()
            if command == Commands.WAYPOINT.value:
                prop: WaypointPropertyWidget = self._get_property(item)
                latitude = prop.latitude.value()
                longitude = prop.longitude.value()
                coord = QGeoCoordinate(latitude, longitude)
                self._map.add_waypoint(index, coord, prop.acceptance_radius.value(), "cyan")
                if last_coord is not None:
                    last_latitude, last_longitude = last_coord
                    self._map.add_line(last_latitude, last_longitude, latitude, longitude)
                index += 1
                last_coord = (latitude, longitude)
            elif command == Commands.TAKEOFF.value:
                pass
            elif command == Commands.LAND.value:
                pass
            elif command == Commands.RETURN_TO_HOME.value:
                pass
            else:
                raise RuntimeError(f"Unknown command: {command}")

    def _get_property(self, item: QListWidgetItem) -> BasePropertyWidget:
        for item_, prop in self._pairs:
            if item_ is item:
                return prop
        else:
            raise RuntimeError()
