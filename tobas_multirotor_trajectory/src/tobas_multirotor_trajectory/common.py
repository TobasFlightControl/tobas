import rospy
import actionlib
from abc import ABC, abstractmethod

from tobas_trajectory_commander.msg import (
    FollowPositionYawTrajectoryAction,
    FollowPositionYawTrajectoryGoal,
    FollowPositionYawTrajectoryResult,
)


class FollowTrajectoryClient(ABC):

    ACTION_NAME = "follow_trajectory_position_yaw"
    WAIT_FOR_SERVER = 5.

    def __init__(self) -> None:
        self._ac = actionlib.SimpleActionClient(
            self.ACTION_NAME,
            FollowPositionYawTrajectoryAction,
        )

    def run(self) -> None:
        if not self._ac.wait_for_server(rospy.Duration(self.WAIT_FOR_SERVER)):
            rospy.logerr(f'Failed to connect to "{self.ACTION_NAME}" action server')
            return

        goal = self._make_goal()
        self._call_action_and_show_result(goal)

    def _call_action_and_show_result(self, goal: FollowPositionYawTrajectoryGoal) -> None:
        rospy.loginfo(f'Sending a goal to "{self.ACTION_NAME}" action.')
        self._ac.send_goal_and_wait(goal)
        rospy.logdebug("Action finished.")

        result: FollowPositionYawTrajectoryResult = self._ac.get_result()
        if result:
            rospy.loginfo(f'Result: {result.error_code}')

    @abstractmethod
    def _make_goal(self) -> FollowPositionYawTrajectoryGoal:
        pass
