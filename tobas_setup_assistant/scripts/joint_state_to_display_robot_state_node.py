#!/usr/bin/env python3

import os.path as osp
import rospy
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayRobotState


class JointStateToDisplayRobotState:
    def __init__(self) -> None:
        self._robot_state = DisplayRobotState()

        self._robot_state_pub = rospy.Publisher(
            "display_robot_state", DisplayRobotState, queue_size=1
        )
        self._joint_state_sub = rospy.Subscriber(
            "joint_states", JointState, self._joint_state_cb, queue_size=1
        )

    def _joint_state_cb(self, js: JointState) -> None:
        self._robot_state.state.joint_state = js
        self._robot_state_pub.publish(self._robot_state)


if __name__ == "__main__":
    node_name = osp.splitext(osp.basename(__file__))[0]
    rospy.init_node(node_name)
    node = JointStateToDisplayRobotState()
    rospy.spin()
