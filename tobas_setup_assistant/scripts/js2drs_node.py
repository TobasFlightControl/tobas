import rclpy
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayRobotState

from tobas_rclpy.utils import init_node


class JointStateToDisplayRobotState:
    def __init__(self) -> None:
        self._robot_state = DisplayRobotState()

        self._robot_state_pub = self.create_publisher("display_robot_state", DisplayRobotState, 1)
        self._joint_state_sub = self.create_subscription("joint_states", JointState, self._joint_state_cb, 1)

    def _joint_state_cb(self, js: JointState) -> None:
        self._robot_state.state.joint_state = js
        self._robot_state_pub.publish(self._robot_state)


if __name__ == "__main__":
    init_node()
    node = JointStateToDisplayRobotState()
    rclpy.spin()
