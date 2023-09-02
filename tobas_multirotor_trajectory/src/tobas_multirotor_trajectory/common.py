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
    WAIT_FOR_SERVER = 3.0  # [s]

    def __init__(self) -> None:
        self._ac = actionlib.SimpleActionClient(
            self.ACTION_NAME,
            FollowPositionYawTrajectoryAction,
        )

        # Ctrl + Cでアクションを止められるようにする
        rospy.on_shutdown(self._on_shutdown)

    def run(self) -> None:
        # ノードの起動直後にrospy.Timeにアクセスすると0が返る可能性があるため，少し待機する
        rospy.sleep(0.1)

        if not self._ac.wait_for_server(rospy.Duration.from_sec(self.WAIT_FOR_SERVER)):
            rospy.logerr(f'Failed to connect to "{self.ACTION_NAME}" action server')
            return

        goal = self._make_goal()
        self._call_action_and_show_result(goal)

    def _call_action_and_show_result(
        self, goal: FollowPositionYawTrajectoryGoal
    ) -> None:
        rospy.loginfo(f'Sending a goal to "{self.ACTION_NAME}" action.')
        self._ac.send_goal_and_wait(goal)
        rospy.logdebug("Action finished.")

        result: FollowPositionYawTrajectoryResult = self._ac.get_result()
        if result:
            rospy.loginfo(f"Result: {result.error_code}")

    def _on_shutdown(self) -> None:
        rospy.loginfo("Program interrupted before completion. Canceling action.")
        self._ac.cancel_goal()

    @abstractmethod
    def _make_goal(self) -> FollowPositionYawTrajectoryGoal:
        pass
