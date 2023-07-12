import rospy
from copy import deepcopy

from tobas_msgs.msg import CommandLevel, PositionYawTrajectoryPoint
from tobas_trajectory_commander.msg import FollowPositionYawTrajectoryGoal

from .common import FollowTrajectoryClient


class FollowTrajectoryClient_UpDown(FollowTrajectoryClient):

    def __init__(self) -> None:
        super().__init__()

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

        # 上下運動
        for _ in range(5):
            point.pos.z += 2.
            point.time_from_start += rospy.Duration.from_sec(5.)
            goal.waypoints.append(deepcopy(point))

            point.pos.z -= 2.
            point.time_from_start += rospy.Duration.from_sec(5.)
            goal.waypoints.append(deepcopy(point))

        # 下降
        point.pos.z = -2.  # 安全のため余分に下げる
        point.time_from_start += rospy.Duration.from_sec(5.)
        goal.waypoints.append(deepcopy(point))

        return goal
