import rospy
import actionlib
from overrides import override
from typing import List
from PyQt5.QtCore import QObject, QThread, pyqtSignal

from tobas_msgs.msg import CommandLevel, TakeoffAction, TakeoffGoal, LandAction, LandGoal, MoveAction, MoveGoal
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

        self._get_gnss_origin_sc = rospy.ServiceProxy(f"/{drone_name}/get_gnss_origin", GetGnssOrigin)

        self._takeoff_ac = actionlib.SimpleActionClient(f"/{drone_name}/takeoff_action", TakeoffAction)
        self._land_ac = actionlib.SimpleActionClient(f"/{drone_name}/land_action", LandAction)
        self._move_ac = actionlib.SimpleActionClient(f"/{drone_name}/move_action", MoveAction)

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
            self._get_gnss_origin_sc.wait_for_service(WAIT_FOR_SERVER)
        except rospy.ROSException:
            self.finished.emit(False, "Get GNSS origin server is not ready.")
            return False

        if not self._takeoff_ac.wait_for_server(rospy.Duration(WAIT_FOR_SERVER)):
            self.finished.emit(False, "Takeoff action server is not ready.")
            return False
        if not self._land_ac.wait_for_server(rospy.Duration(WAIT_FOR_SERVER)):
            self.finished.emit(False, "Land action server is not ready.")
            return False
        if not self._move_ac.wait_for_server(rospy.Duration(WAIT_FOR_SERVER)):
            self.finished.emit(False, "Move action server is not ready.")
            return False

        return True

    def _execute_command(self, command) -> bool:
        if isinstance(command, WaypointProperty):
            return self._execute_waypoint(command)
        elif isinstance(command, TakeoffProperty):
            return self._execute_takeoff(command)
        elif isinstance(command, LandProperty):
            return self._execute_land(command)
        elif isinstance(command, RTHProperty):
            return self._execute_rth(command)
        else:
            raise RuntimeError(f"Unknown command type: {command.__class__.__name__}")

    def _execute_waypoint(self, command: WaypointProperty) -> bool:
        rospy.loginfo('Executing "Waypoint" mission.')

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
        while not self._move_ac.wait_for_result(rospy.Duration(self.WAIT_FOR_RESULT)):
            if self._stop_requested:
                self._move_ac.cancel_goal()
                return False

        if self._move_ac.get_state() != actionlib.GoalStatus.SUCCEEDED:
            self.finished.emit(False, self._move_ac.get_goal_status_text())
            return False

        return True

    def _execute_takeoff(self, command: TakeoffProperty) -> bool:
        rospy.loginfo('Executing "Takeoff" mission.')

        # ゴールを作成
        goal = TakeoffGoal()
        goal.level.data = self.COMMAND_LEVEL
        goal.target_altitude = command.altitude
        goal.duration = command.duration
        goal.timeout = self.COMMAND_TIMEOUT

        # アクションを実行
        self._takeoff_ac.send_goal(goal)

        # 終了フラグを監視しながら待機
        while not self._takeoff_ac.wait_for_result(rospy.Duration(self.WAIT_FOR_RESULT)):
            if self._stop_requested:
                self._takeoff_ac.cancel_goal()
                return False

        if self._takeoff_ac.get_state() != actionlib.GoalStatus.SUCCEEDED:
            self.finished.emit(False, self._takeoff_ac.get_goal_status_text())
            return False

        return True

    def _execute_land(self, command: LandProperty) -> bool:
        rospy.loginfo('Executing "Land" mission.')

        # ゴールを作成
        goal = LandGoal()
        goal.level.data = self.COMMAND_LEVEL

        # アクションを実行
        self._land_ac.send_goal(goal)

        # 終了フラグを監視しながら待機
        while not self._land_ac.wait_for_result(rospy.Duration(self.WAIT_FOR_RESULT)):
            if self._stop_requested:
                self._land_ac.cancel_goal()
                return False

        if self._land_ac.get_state() != actionlib.GoalStatus.SUCCEEDED:
            self.finished.emit(False, self._land_ac.get_goal_status_text())
            return False

        return True

    def _execute_rth(self, command: RTHProperty) -> bool:
        rospy.loginfo('Executing "Return to Home" mission.')

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
        while not self._move_ac.wait_for_result(rospy.Duration(self.WAIT_FOR_RESULT)):
            if self._stop_requested:
                self._move_ac.cancel_goal()
                return False

        if self._move_ac.get_state() != actionlib.GoalStatus.SUCCEEDED:
            self.finished.emit(False, self._move_ac.get_goal_status_text())
            return False

        return True
