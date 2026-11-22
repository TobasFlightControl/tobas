# User Code (Python)

This section assumes you know the basics of Python and ROS 2.
To learn ROS 2, see
<a href=https://docs.ros.org/en/jazzy/Tutorials.html target="_blank">Tutorials | ROS 2 Documentation</a>
for tutorials.

Among the ROS packages in a Tobas project created with Setup Assistant (e.g., tobas_f450.TBS),
the user Python package (e.g., tobas_f450_user_py) is a Python package you can freely edit.
It contains the following three launch files:

- `common.launch.py`: Runs on both real hardware and in simulation.
- `real.launch.py`: Runs only on real hardware.
- `gazebo.launch.py`: Runs only in simulation.

As an example, create a Python node that checks GNSS status and publishes a message when a 3D fix is available.
Edit `tobas_f450_user_py/tobas_f450_user_py/user_node.py` as follows.

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from tobas_msgs.msg import Message
from tobas_msgs.msg import Gnss


class UserNode(Node):
    def __init__(self) -> None:
        super().__init__("gnss_state_checker")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE

        self._message_pub = self.create_publisher(Message, "message", qos)
        self._gnss_sub = self.create_subscription(Gnss, "gnss", self._gnss_callback, qos)

    def _gnss_callback(self, gnss: Gnss) -> None:
        message = Message()
        message.header.stamp = gnss.header.stamp
        message.name = self.get_name()

        if gnss.fix_type == Gnss.FIX_3D:
            message.level = Message.LEVEL_INFO
            message.message = "GNSS 3D Fix"
            self._message_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UserNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
```

Configure the node to start automatically.
Uncomment the `add_action` section in `tobas_f450_user_py/launch/common.launch.py`.

```python
# Do not delete or rename this file because it is executed in tobas_f450_config/common_interface.launch.py.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Please add the nodes that run both on real hardware and in simulation.

    ld.add_action(
        Node(
            package="tobas_f450_user_py",
            executable="user_node",
        )
    )

    return ld
```

When you start the simulation from the GCS, messages appear in the `Control System` console.

![console](../../assets/user_code_py/console.png)

See [ROS API](./ros_api.md) for API details.
