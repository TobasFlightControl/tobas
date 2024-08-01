import rclpy
from rclpy.duration import Duration
from copy import deepcopy
from overrides import override

from tobas_msgs.msg import CommandLevel, PositionYawTrajectoryPoint
from tobas_trajectory_commander.msg import FollowPositionYawTrajectoryGoal

from .common import FollowTrajectoryClient


class FollowTrajectoryClient_Square(FollowTrajectoryClient):
    EDGE_LENGTH = 3.0  # [m]

    def __init__(self) -> None:
        super().__init__()

    @override
    def _make_goal(self) -> FollowPositionYawTrajectoryGoal:
        goal = FollowPositionYawTrajectoryGoal()
        goal.level.data = CommandLevel.NORMAL
        goal.degree = 1

        point = PositionYawTrajectoryPoint()

        # 初期状態
        goal.waypoints.append(deepcopy(point))

        # 上昇
        point.pos.z = 2.0
        point.time_from_start += Duration.from_sec(5.0)
        goal.waypoints.append(deepcopy(point))

        # 正方形
        point.pos.x = self.EDGE_LENGTH
        point.time_from_start += Duration.from_sec(5.0)
        goal.waypoints.append(deepcopy(point))

        point.pos.y = self.EDGE_LENGTH
        point.time_from_start += Duration.from_sec(5.0)
        goal.waypoints.append(deepcopy(point))

        point.pos.x = 0.0
        point.time_from_start += Duration.from_sec(5.0)
        goal.waypoints.append(deepcopy(point))

        point.pos.y = 0.0
        point.time_from_start += Duration.from_sec(5.0)
        goal.waypoints.append(deepcopy(point))

        # 下降
        point.pos.z = -2.0  # 安全のため余分に下げる
        point.time_from_start += Duration.from_sec(5.0)
        goal.waypoints.append(deepcopy(point))

        return goal
