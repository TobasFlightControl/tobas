from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...gcs import GroundControlStationWidget

import rospy
import actionlib
from overrides import override
from functools import partial
from typing import Tuple, List
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtPositioning import QGeoCoordinate

from tobas_std_tools_py.threading import KillableThread
from tobas_rqt_tools.widgets import ListWidget, StackedWidget
from tobas_rqt_tools.messages import q_info, q_warn, q_error
from tobas_tools_py.drone import Drone
from tobas_msgs.msg import CommandLevel, TakeoffAction, TakeoffGoal, LandAction, LandGoal, MoveAction, MoveGoal
from tobas_msgs.srv import GetGnssOrigin, GetGnssOriginRequest, GetGnssOriginResponse

from ...common import NOT_IMPLEMENTED
from ..base import BaseAppWidget
from .map_widget import MapWidget
from .add_command_dialog import Commands, AddCommandDialog
from .command_properties import *


class MissionPlannerWidget(BaseAppWidget):
    NAME = "Mission Planner"

    BUTTON_WIDTH = 100
    BUTTON_HEIGHT = 40
    WAIT_FOR_SERVER = 1.0  # [s]
    COMMAND_LEVEL = CommandLevel.NORMAL
    COMMAND_TIMEOUT = 10.0  # [s]  # TODO: ユーザが設定できるようにする

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
        self._execute_button.setEnabled(False)
        button_cols.addWidget(self._execute_button)

        self._cancel_button = QPushButton("Cancel")
        self._cancel_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._cancel_button.setEnabled(False)
        button_cols.addWidget(self._cancel_button)

        mission_cols = QHBoxLayout()
        rows.addLayout(mission_cols)

        self._command_list = ListWidget()
        self._command_list.setSelectionMode(QListWidget.SingleSelection)
        self._command_list.setDragDropMode(QListWidget.InternalMove)
        mission_cols.addWidget(self._command_list)

        self._properties = StackedWidget()
        self._properties.setStyleSheet("QStackedWidget { border: 1px solid black; background-color: white; }")
        mission_cols.addWidget(self._properties)

        self._pairs: List[Tuple[QListWidgetItem, BasePropertyWidget]] = []
        self._mission_thread = KillableThread()

        self._takeoff_ac = None
        self._land_ac = None
        self._move_ac = None
        self._get_gnss_origin_sc = None

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
        self._takeoff_ac = actionlib.SimpleActionClient(f"/{self._drone.drone_name}/takeoff_action", TakeoffAction)
        self._land_ac = actionlib.SimpleActionClient(f"/{self._drone.drone_name}/land_action", LandAction)
        self._move_ac = actionlib.SimpleActionClient(f"/{self._drone.drone_name}/move_action", MoveAction)

        self._get_gnss_origin_sc = rospy.ServiceProxy(f"/{self._drone.drone_name}/get_gnss_origin", GetGnssOrigin)

        self._execute_button.setEnabled(True)

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
        self._map.clear()
        self._command_list.clear()
        self._properties.clear()
        self._pairs.clear()

    @pyqtSlot()
    def _on_execute_button_clicked(self) -> None:
        # ミッションが設定されているかどうかを確認
        if self._command_list.count() == 0:
            q_error(self._main, "Mission is empty.")
            return

        # TODO: 有効なミッションかどうかを確認 (Takeoff後にTakeoffはダメとか)

        # 各サーバとの接続を確認
        if not self._check_server_connections(self):
            return

        # 実行モードに切り替える
        self._set_execute_mode()

        # ユーザ操作をブロックしないように別スレッドでミッションを実行
        self._mission_thread = KillableThread(target=self._execute_mission)
        self._mission_thread.start()

    @pyqtSlot()
    def _on_cancel_button_clicked(self) -> None:
        # アクションをキャンセル
        # TODO: とりあえず全てキャンセルしているが，現在実行中のアクションのみキャンセルする．
        self._takeoff_ac.cancel_goal()
        self._land_ac.cancel_goal()
        self._move_ac.cancel_goal()

        # ミッションを実行しているスレッドを落とす
        if not self._mission_thread.kill():
            q_error(self._main, "Failed to terminate the thread for mission execution.")
            return

        # 編集モードに切り替える
        self._set_edit_mode()

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
        if self._mission_thread.is_alive():
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

    def _execute_mission(self) -> None:
        for item in self._command_list:
            prop = self._get_property(item)
            if not self._execute_command(prop):
                return

        q_info(self._main, "The mission is completed.")
        self._set_edit_mode()

    def _execute_command(self, prop: BasePropertyWidget) -> bool:
        if isinstance(prop, WaypointPropertyWidget):
            return self._execute_waypoint(prop)
        elif isinstance(prop, TakeoffPropertyWidget):
            return self._execute_takeoff(prop)
        elif isinstance(prop, LandPropertyWidget):
            return self._execute_land(prop)
        elif isinstance(prop, RTHPropertyWidget):
            return self._execute_rth(prop)
        else:
            raise RuntimeError(f"Unknown command type: {prop.__class__.__name__}")

    def _execute_waypoint(self, prop: WaypointPropertyWidget) -> bool:
        # ゴールを作成
        goal = MoveGoal()
        goal.level.data = self.COMMAND_LEVEL
        goal.target_latitude = prop.latitude.value()
        goal.target_longitude = prop.longitude.value()
        goal.target_altitude = prop.altitude.value()
        goal.acceptance_radius = prop.acceptance_radius.value()
        goal.duration = prop.duration.value()
        goal.timeout = self.COMMAND_TIMEOUT

        # アクションを実行
        self._move_ac.send_goal_and_wait(goal)

        if self._move_ac.get_state() != actionlib.GoalStatus.SUCCEEDED:
            q_error(self._main, self._move_ac.get_goal_status_text())
            return False

        return True

    def _execute_takeoff(self, prop: TakeoffPropertyWidget) -> bool:
        # ゴールを作成
        goal = TakeoffGoal()
        goal.level.data = self.COMMAND_LEVEL
        goal.target_altitude = prop.altitude.value()
        goal.duration = prop.duration.value()
        goal.timeout = self.COMMAND_TIMEOUT

        # アクションを実行
        self._takeoff_ac.send_goal_and_wait(goal)

        if self._takeoff_ac.get_state() != actionlib.GoalStatus.SUCCEEDED:
            q_error(self._main, self._takeoff_ac.get_goal_status_text())
            return False

        return True

    def _execute_land(self, prop: LandPropertyWidget) -> bool:
        # ゴールを作成
        goal = LandGoal()
        goal.level.data = self.COMMAND_LEVEL

        # アクションを実行
        self._land_ac.send_goal_and_wait(goal)

        if self._land_ac.get_state() != actionlib.GoalStatus.SUCCEEDED:
            q_error(self._main, self._land_ac.get_goal_status_text())
            return False

        return True

    def _execute_rth(self, prop: RTHPropertyWidget) -> bool:
        # ホームポジションの経緯度を取得
        res: GetGnssOriginResponse = self._get_gnss_origin_sc.call(GetGnssOriginRequest())
        if not res.success:
            q_error(self._main, f"Failed to get GNSS origin: {res.message}")

        # ゴールを作成
        goal = MoveGoal()
        goal.level.data = self.COMMAND_LEVEL
        goal.target_latitude = res.latitude
        goal.target_longitude = res.longitude
        goal.target_altitude = prop.altitude.value()
        goal.acceptance_radius = prop.acceptance_radius.value()
        goal.duration = prop.duration.value()
        goal.timeout = self.COMMAND_TIMEOUT

        # アクションを実行
        self._move_ac.send_goal_and_wait(goal)

        if self._move_ac.get_state() != actionlib.GoalStatus.SUCCEEDED:
            q_error(self._main, self._move_ac.get_goal_status_text())
            return False

        return True

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

    def _check_server_connections(self) -> bool:
        if not self._takeoff_ac.wait_for_server(rospy.Duration(self.WAIT_FOR_SERVER)):
            q_error(self._main, "Takeoff action server is not ready.")
            return False
        if not self._land_ac.wait_for_server(rospy.Duration(self.WAIT_FOR_SERVER)):
            q_error(self._main, "Land action server is not ready.")
            return False
        if not self._move_ac.wait_for_server(rospy.Duration(self.WAIT_FOR_SERVER)):
            q_error(self._main, "Move action server is not ready.")
            return False

        try:
            self._get_gnss_origin_sc.wait_for_service(self.WAIT_FOR_SERVER)
        except rospy.ROSException:
            q_error(self._main, "Get GNSS origin server is not ready.")
            return False
