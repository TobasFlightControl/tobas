from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

import os.path as osp
import rclpy
import shutil
from bisect import bisect_left
from typing import override
from functools import partial
from typing import Tuple, List
from pathlib import Path
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import (
    QDialog,
    QListWidget,
    QListWidgetItem,
    QVBoxLayout,
    QHBoxLayout,
    QSizePolicy,
)
from PyQt5.QtPositioning import QGeoCoordinate

from tobas_std_tools_py.algorithm import cumsum
from tobas_rqt_tools.widgets import ListWidget, StackedWidget
from tobas_rqt_tools.messages import q_info, q_warn, q_error, yes_or_no, QMessageLevel
from tobas_tools_py.drone import Drone
from tobas_msgs.msg import Gps

from ...common import CONFIG_PKG_NOT_LOADED, NOT_IMPLEMENTED, WAIT_FOR_SERVER
from ..base import BaseAppWidget
from .map_widget import MapWidget
from .add_command_dialog import Commands, AddCommandDialog
from .mission_execution_thread import MissionExecutionThread
from .command_properties import *


class MissionPlannerWidget(BaseAppWidget):
    NAME = "Mission Planner"

    BUTTON_WIDTH = 100
    BUTTON_HEIGHT = 40

    CACHE_DIR_ONLINE = osp.expanduser("~/.cache/tobas/tiles/online/")
    CACHE_DIR_OFFLINE = osp.expanduser("~/.cache/tobas/tiles/offline/")
    CACHE_MAX_SIZE = 1 << 30  # 1GiB

    def __init__(self, main: GroundControlStationWidget, drone: Drone) -> None:
        super().__init__(main, drone)

        rows = QVBoxLayout()
        self.setLayout(rows)

        self._map = MapWidget()
        self._map.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._map.waypoint_moved.connect(self._on_waypoint_moved)
        rows.addWidget(self._map)

        button_cols = QHBoxLayout()
        rows.addLayout(button_cols)

        self._load_button = QPushButton("Load")
        self._load_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._load_button.clicked.connect(self._on_load_button_clicked)
        button_cols.addWidget(self._load_button)

        self._save_button = QPushButton("Save")
        self._save_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._save_button.clicked.connect(self._on_save_button_clicked)
        button_cols.addWidget(self._save_button)

        self._add_button = QPushButton("Add")
        self._add_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._add_button.clicked.connect(self._on_add_button_clicked)
        button_cols.addWidget(self._add_button)

        self._clear_button = QPushButton("Clear")
        self._clear_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._clear_button.clicked.connect(self._on_clear_button_clicked)
        button_cols.addWidget(self._clear_button)

        self._cache_button = QPushButton("Cache Map")
        self._cache_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._cache_button.clicked.connect(self._on_cache_button_clicked)
        button_cols.addWidget(self._cache_button)

        button_cols.addStretch()

        self._execute_button = QPushButton("Execute")
        self._execute_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._execute_button.clicked.connect(self._on_execute_button_clicked)
        button_cols.addWidget(self._execute_button)

        self._cancel_button = QPushButton("Cancel")
        self._cancel_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._cancel_button.setEnabled(False)
        self._cancel_button.clicked.connect(self._on_cancel_button_clicked)
        button_cols.addWidget(self._cancel_button)

        self._focus_button = QPushButton("Focus")
        self._focus_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._focus_button.clicked.connect(self._on_focus_button_clicked)
        button_cols.addWidget(self._focus_button)

        mission_cols = QHBoxLayout()
        rows.addLayout(mission_cols)

        self._command_list = ListWidget()
        self._command_list.setSelectionMode(QListWidget.SingleSelection)
        self._command_list.setDragDropMode(QListWidget.InternalMove)
        self._command_list.itemClicked.connect(self._on_list_item_changed)
        self._command_list.item_moved.connect(self._on_list_item_changed)
        mission_cols.addWidget(self._command_list)

        self._properties = StackedWidget()
        self._properties.setStyleSheet("QStackedWidget { border: 1px solid black; background-color: white; }")
        mission_cols.addWidget(self._properties)

        self._pairs: List[Tuple[QListWidgetItem, BasePropertyWidget]] = []
        self._mission_thread = QThread()

    @override
    def update_internal_data_structures(self) -> None:
        pass

    @pyqtSlot()
    def _on_load_button_clicked(self) -> None:
        q_warn(self._main, NOT_IMPLEMENTED)  # TODO

    @pyqtSlot()
    def _on_save_button_clicked(self) -> None:
        q_warn(self._main, NOT_IMPLEMENTED)  # TODO

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
        if not yes_or_no(self._main, "Do you want to clear all the commands?", QMessageLevel.WARN):
            return

        self._map.clear()
        self._command_list.clear()
        self._properties.clear()
        self._pairs.clear()

    @pyqtSlot()
    def _on_cache_button_clicked(self) -> None:
        if not yes_or_no(
            self._main,
            "Do you want to cache map tiles to offline storage?",
            QMessageLevel.WARN,
        ):
            return

        # 確認用のディレクトリパスをPathオブジェクトに変換
        dir_from = Path(self.CACHE_DIR_ONLINE)
        dir_to = Path(self.CACHE_DIR_OFFLINE)

        if not dir_to.exists():
            dir_to.mkdir()

        # 全てのPNGファイルをコピー
        for file_from in dir_from.glob("*.png"):
            file_to = dir_to / file_from.name
            shutil.copy(file_from, file_to)  # ファイルをコピー (最終変更時刻はコピーした瞬間の時刻になる)

        # ディレクトリの合計サイズがリミット未満になるまでファイルを削除
        files = list(dir_to.glob("*.png"))
        files.sort(key=lambda x: x.stat().st_mtime_ns, reverse=True)  # 最終変更時刻が新しい順に並べかえる
        sizes = [file.stat().st_size for file in files]  # 全てのファイルのサイズ[bits]を計算
        sizes_cs = cumsum(sizes)  # ファイルサイズの累積和
        last_alive_idx = bisect_left(sizes_cs, self.CACHE_MAX_SIZE)  # 新しい方から数えて最大サイズを超える位置
        for i in range(last_alive_idx, len(files)):
            files[i].unlink()  # ファイルを削除

        q_info(self._main, f"Map tiles are cached to {self.CACHE_DIR_OFFLINE}.")

    @pyqtSlot()
    def _on_execute_button_clicked(self) -> None:
        if not yes_or_no(self._main, "Do you want to execute the mission?", QMessageLevel.WARN):
            return

        if not self._main.pkg_loaded():
            q_error(self._main, CONFIG_PKG_NOT_LOADED)
            return

        # ミッションが設定されているかどうかを確認
        if self._command_list.count() == 0:
            q_error(self._main, "Mission is empty.")
            return

        # ミッションデータを抽出
        mission_commands = self._create_mission_commands()

        # TODO: 有効なミッションかどうかを確認 (Takeoff後にTakeoffはダメとか)

        # 実行モードに切り替える
        self._set_execute_mode()

        # ユーザ操作をブロックしないように別スレッドでミッションを実行
        self._mission_thread = MissionExecutionThread(self, self._drone.name, mission_commands)
        self._mission_thread.finished.connect(self._on_mission_finished)
        self._mission_thread.start()

    @pyqtSlot()
    def _on_cancel_button_clicked(self) -> None:
        if not yes_or_no(self._main, "Do you want to cancel the mission?", QMessageLevel.WARN):
            return

        if not self._main.pkg_loaded():
            q_error(self._main, CONFIG_PKG_NOT_LOADED)
            return

        # ミッションを停止
        self._mission_thread.stop()

        # 編集モードに切り替える
        self._set_edit_mode()

        q_info(self._main, "The mission is canceled.")

    @pyqtSlot()
    def _on_focus_button_clicked(self) -> None:
        if not self._main.pkg_loaded():
            q_error(self._main, CONFIG_PKG_NOT_LOADED)
            return

        try:
            gps: Gps = rclpy.wait_for_message(f"{self._drone.name}/gps", Gps, WAIT_FOR_SERVER)
        except rclpy.ROSException:
            q_error(self._main, "Failed to get GNSS message.")
            return

        self._map.set_center(gps.latitude, gps.longitude)

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

    @pyqtSlot()
    def _on_property_value_changed(self) -> None:
        self._update_map()

    @pyqtSlot(int, float, float)
    def _on_waypoint_moved(self, index: int, latitude: float, longitude: float) -> None:
        if self._mission_thread.isRunning():
            q_warn(self._main, "You cannot edit the mission while executing it.")
            self._update_map()
            return

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

    @pyqtSlot(bool, str)
    def _on_mission_finished(self, success: bool, message: str) -> None:
        if success:
            q_info(self._main, "The mission is completed.")
        else:
            q_error(self._main, message)

        self._set_edit_mode()

    def _set_execute_mode(self) -> None:
        """各ウィジェットを実行モードに切り替える．"""
        self._load_button.setEnabled(False)
        self._save_button.setEnabled(False)
        self._add_button.setEnabled(False)
        self._clear_button.setEnabled(False)
        self._execute_button.setEnabled(False)
        self._cancel_button.setEnabled(True)

        self._command_list.setDragDropMode(QListWidget.NoDragDrop)
        self._properties.setEnabled(False)

    def _set_edit_mode(self) -> None:
        """各ウィジェットを編集モードに切り替える．"""
        self._load_button.setEnabled(True)
        self._save_button.setEnabled(True)
        self._add_button.setEnabled(True)
        self._clear_button.setEnabled(True)
        self._execute_button.setEnabled(True)
        self._cancel_button.setEnabled(False)

        self._command_list.setDragDropMode(QListWidget.InternalMove)
        self._properties.setEnabled(True)

    def _update_properties(self) -> None:
        """選択されているリストアイテムに基づいてプロパティの表示を更新．"""
        if self._command_list.count() == 0:
            return

        # 選択されているアイテムを取得
        selected_item = self._command_list.selected_item()
        if selected_item is None:
            # 何も選択されていなければ強制的に最初の要素を選択する
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
        selected_item = self._command_list.selected_item()

        for item in self._command_list:
            command = item.text()

            if command == Commands.WAYPOINT.value:
                prop: WaypointPropertyWidget = self._get_property(item)
                latitude = prop.latitude.value()
                longitude = prop.longitude.value()
                coord = QGeoCoordinate(latitude, longitude)

                point_color = "orange" if item is selected_item else "cyan"
                self._map.add_waypoint(index, coord, prop.acceptance_radius.value(), point_color)

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

    def _create_mission_commands(self) -> List:
        res = []
        for item in self._command_list:
            prop = self._get_property(item)
            res.append(prop.get_data())
        return res
