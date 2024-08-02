import rclpy
from rclpy.duration import Duration
from copy import deepcopy
from typing import override

from tobas_msgs.msg import CommandLevel, PositionYawTrajectoryPoint
from tobas_trajectory_commander.msg import FollowPositionYawTrajectoryGoal

from .common import FollowTrajectoryClient


class FollowTrajectoryClient_Hover(FollowTrajectoryClient):
    def __init__(self) -> None:
        super().__init__()

    @override
    def _make_goal(self) -> FollowPositionYawTrajectoryGoal:
        goal = FollowPositionYawTrajectoryGoal()
        goal.level.data = CommandLevel.NORMAL
        goal.degree = 1  # 線形補間

        point = PositionYawTrajectoryPoint()

        # 初期状態
        goal.waypoints.append(deepcopy(point))

        # 上昇
        point.pos.z = 2.0
        point.time_from_start += Duration.from_sec(5.0)
        goal.waypoints.append(deepcopy(point))

        # 待機
        point.time_from_start += Duration.from_sec(10.0)
        goal.waypoints.append(deepcopy(point))

        # 下降
        point.pos.z = -2.0  # 安全のため余分に下げる
        point.time_from_start += Duration.from_sec(5.0)
        goal.waypoints.append(deepcopy(point))

        return goal
