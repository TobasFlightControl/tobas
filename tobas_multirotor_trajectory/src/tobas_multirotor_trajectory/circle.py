import rospy
import numpy as np
from copy import deepcopy
from overrides import overrides

from tobas_msgs.msg import CommandLevel, PositionYawTrajectoryPoint
from tobas_trajectory_commander.msg import FollowPositionYawTrajectoryGoal

from .common import FollowTrajectoryClient


class FollowTrajectoryClient_Circle(FollowTrajectoryClient):

    RADIUS = 3.   # [m]
    PERIOD = 10.  # [s]

    def __init__(self) -> None:
        super().__init__()

    @overrides
    def _make_goal(self) -> FollowPositionYawTrajectoryGoal:
        goal = FollowPositionYawTrajectoryGoal()
        goal.level.data = CommandLevel.NORMAL
        goal.degree = 1

        point = PositionYawTrajectoryPoint()

        # 初期状態
        goal.waypoints.append(deepcopy(point))

        # 上昇
        point.pos.z = 2.
        point.time_from_start += rospy.Duration.from_sec(5.)
        goal.waypoints.append(deepcopy(point))

        # 円運動
        circle_start_time = point.time_from_start
        ts = np.linspace(0.,  self.PERIOD, 100)[1:]
        for t in ts:
            theta = 2 * np.pi * t / self.PERIOD
            point.pos.x = self.RADIUS * np.sin(theta)
            point.pos.y = self.RADIUS * (1 - np.cos(theta))
            point.time_from_start = circle_start_time + rospy.Duration.from_sec(t)
            goal.waypoints.append(deepcopy(point))

        # 下降
        point.pos.z = -2.  # 安全のため余分に下げる
        point.time_from_start += rospy.Duration.from_sec(5.)
        goal.waypoints.append(deepcopy(point))

        return goal
