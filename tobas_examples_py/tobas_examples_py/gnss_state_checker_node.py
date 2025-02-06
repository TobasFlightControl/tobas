import rclpy
import rclpy.node
import rclpy.qos

from tobas_std_msgs.msg import Message
from tobas_msgs.msg import Gnss


class GnssStateCheckerNode(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__("gnss_state_checker")

        qos = rclpy.qos.QoSProfile(depth=1)
        qos.reliability = rclpy.qos.ReliabilityPolicy.BEST_EFFORT
        qos.durability = rclpy.qos.DurabilityPolicy.VOLATILE

        self._message_pub = self.create_publisher(Message, "message", qos)
        self._gnss_sub = self.create_subscription(Gnss, "gnss", self._gnss_callback, qos)

    def _gnss_callback(self, gnss: Gnss) -> None:
        message = Message()
        message.stamp = gnss.header.stamp
        message.name = self.get_name()

        if gnss.fix_type == Gnss.FIX_3D:
            message.level = Message.LEVEL_INFO
            message.message = "GNSS Fix"
        else:
            message.level = Message.LEVEL_WARN
            message.message = "GNSS No Fix"

        self._message_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GnssStateCheckerNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
