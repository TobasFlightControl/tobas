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

        self._command_list.itemClicked.connect(self._on_list_item_clicked)

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
        prop.delete_button_clicked.connect(partial(self._on_delete_button_clicked, target_item=item, target_prop=prop))

        self._pairs.append((item, prop))

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
    def _on_delete_button_clicked(self, target_item: QListWidgetItem, target_prop: BasePropertyWidget) -> None:
        self._command_list.remove(target_item)
        self._properties.removeWidget(target_prop)

        for item, prop in self._pairs:
            if item is target_item and prop is target_prop:
                self._pairs.remove((item, prop))
                return
        else:
            raise RuntimeError("Failed to find the target item in the set.")

    @pyqtSlot(QListWidgetItem)
    def _on_list_item_clicked(self, clicked_item: QListWidgetItem):
        for item, prop in self._pairs:
            if item is clicked_item:
                self._properties.setCurrentWidget(prop)
                return
        else:
            raise RuntimeError("Failed to find the clicked item in the set.")
