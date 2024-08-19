import rclpy
from rclpy.duration import Duration
import actionlib
from typing import override
from typing import List
from PyQt5.QtCore import QObject, QThread, pyqtSignal

from tobas_tools_py.constants import Service, Action
from tobas_msgs.msg import (
    CommandLevel,
    TakeoffAction,
    TakeoffGoal,
    LandAction,
    LandGoal,
    MoveAction,
    MoveGoal,
)
from tobas_msgs.srv import GetGnssOrigin, GetGnssOriginRequest, GetGnssOriginResponse

from ...common import WAIT_FOR_SERVER
from .structs import *


class MissionExecutionThread(QThread):
    COMMAND_LEVEL = CommandLevel.NORMAL
    COMMAND_TIMEOUT = 10.0  # [s]  # TODO: ユーザが設定できるようにする
    WAIT_FOR_RESULT = 0.1  # [s]

    # 実行結果を返すためのシグナル．
    # NOTE: QThreadでGUIを使うとメインスレッドを壊す恐れがあるため，シグナルスロット以外は使用しない．
    finished = pyqtSignal(bool, str)

    def __init__(self, parent: QObject, drone_name: str, commands: List) -> None:
        super().__init__(parent)

        self._commands = commands
        self._stop_requested = False

        self._get_gnss_origin_sc = self._node.create_client(f"/{drone_name}/{Service.GET_GNSS_ORIGIN}", GetGnssOrigin)

        self._takeoff_ac = actionlib.SimpleActionClient(f"/{drone_name}/{Action.TAKEOFF}", TakeoffAction)
        self._land_ac = actionlib.SimpleActionClient(f"/{drone_name}/{Action.LAND}", LandAction)
        self._move_ac = actionlib.SimpleActionClient(f"/{drone_name}/{Action.MOVE}", MoveAction)

    @override
    def run(self) -> None:
        # 各サーバとの接続を確認
        if not self._check_server_connections():
            return

        # 前から順にコマンドを実行
        for command in self._commands:
            if not self._execute_command(command):
                return

        self.finished.emit(True, "")

    def stop(self) -> None:
        """
        実行中の関数に割り込んでミッションの終了フラグを立てる．

        NOTE
        ----------
        QThread.terminate()はクラッシュの恐れがあるため呼ばない．
        """
        self._stop_requested = True

    def _check_server_connections(self) -> bool:
        try:
            self._get_gnss_origin_sc.service_is_ready(WAIT_FOR_SERVER)
        except rclpy.ROSException:
            self.finished.emit(False, "Get GNSS origin server is not ready.")
            return False

        if not self._takeoff_ac.wait_for_server(Duration(WAIT_FOR_SERVER)):
            self.finished.emit(False, "Takeoff action server is not ready.")
            return False
        if not self._land_ac.wait_for_server(Duration(WAIT_FOR_SERVER)):
            self.finished.emit(False, "Land action server is not ready.")
            return False
        if not self._move_ac.wait_for_server(Duration(WAIT_FOR_SERVER)):
            self.finished.emit(False, "Move action server is not ready.")
            return False

        return True

    def _execute_command(self, command) -> bool:
        self.get_logger().info(f"Executing command: {command}")

        if isinstance(command, Waypoint):
            return self._execute_waypoint(command)
        elif isinstance(command, Takeoff):
            return self._execute_takeoff(command)
        elif isinstance(command, Land):
            return self._execute_land(command)
        elif isinstance(command, ReturnToHome):
            return self._execute_rth(command)
        else:
            raise RuntimeError(f"Unknown command type: {command.__class__.__name__}")

    def _execute_waypoint(self, command: Waypoint) -> bool:
        # ゴールを作成
        goal = MoveGoal()
        goal.level.data = self.COMMAND_LEVEL
        goal.target_latitude = command.latitude
        goal.target_longitude = command.longitude
        goal.target_altitude = command.altitude
        goal.acceptance_radius = command.acceptance_radius
        goal.duration = command.duration
        goal.timeout = self.COMMAND_TIMEOUT

        # アクションを実行
        self._move_ac.send_goal(goal)

        # 終了フラグを監視しながら待機
        while not self._move_ac.wait_for_result(Duration(self.WAIT_FOR_RESULT)):
            if self._stop_requested:
                self._move_ac.cancel_goal()
                return False

        if self._move_ac.get_state() != actionlib.GoalStatus.SUCCEEDED:
            self.finished.emit(False, self._move_ac.get_goal_status_text())
            return False

        return True

    def _execute_takeoff(self, command: Takeoff) -> bool:
        # ゴールを作成
        goal = TakeoffGoal()
        goal.level.data = self.COMMAND_LEVEL
        goal.target_altitude = command.altitude
        goal.altitude_tolerance = command.altitude_tolerance
        goal.duration = command.duration
        goal.timeout = self.COMMAND_TIMEOUT

        # アクションを実行
        self._takeoff_ac.send_goal(goal)

        # 終了フラグを監視しながら待機
        while not self._takeoff_ac.wait_for_result(Duration(self.WAIT_FOR_RESULT)):
            if self._stop_requested:
                self._takeoff_ac.cancel_goal()
                return False

        if self._takeoff_ac.get_state() != actionlib.GoalStatus.SUCCEEDED:
            self.finished.emit(False, self._takeoff_ac.get_goal_status_text())
            return False

        return True

    def _execute_land(self, command: Land) -> bool:
        # ゴールを作成
        goal = LandGoal()
        goal.level.data = self.COMMAND_LEVEL

        # アクションを実行
        self._land_ac.send_goal(goal)

        # 終了フラグを監視しながら待機
        while not self._land_ac.wait_for_result(Duration(self.WAIT_FOR_RESULT)):
            if self._stop_requested:
                self._land_ac.cancel_goal()
                return False

        if self._land_ac.get_state() != actionlib.GoalStatus.SUCCEEDED:
            self.finished.emit(False, self._land_ac.get_goal_status_text())
            return False

        return True

    def _execute_rth(self, command: ReturnToHome) -> bool:
        # ホームポジションの経緯度を取得
        res: GetGnssOriginResponse = self._get_gnss_origin_sc.call(GetGnssOriginRequest())
        if not res.success:
            self.finished.emit(False, f"Failed to get GNSS origin: {res.message}")

        # ゴールを作成
        goal = MoveGoal()
        goal.level.data = self.COMMAND_LEVEL
        goal.target_latitude = res.latitude
        goal.target_longitude = res.longitude
        goal.target_altitude = command.altitude
        goal.acceptance_radius = command.acceptance_radius
        goal.duration = command.duration
        goal.timeout = self.COMMAND_TIMEOUT

        # アクションを実行
        self._move_ac.send_goal(goal)

        # 終了フラグを監視しながら待機
        while not self._move_ac.wait_for_result(Duration(self.WAIT_FOR_RESULT)):
            if self._stop_requested:
                self._move_ac.cancel_goal()
                return False

        if self._move_ac.get_state() != actionlib.GoalStatus.SUCCEEDED:
            self.finished.emit(False, self._move_ac.get_goal_status_text())
            return False

        return True
