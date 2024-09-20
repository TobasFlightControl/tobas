import rclpy
from rclpy.node import Node
from tobas_std_msgs.msg import Message
from tobas_msgs.msg import Gps


class GnssStateCheckerNode(Node):
    def __init__(self) -> None:
        super().__init__("gnss_state_checker")

        self._message_pub = self.create_publisher(Message, "message", 1)
        self._gps_sub = self.create_subscription(Gps, "gps", self._gps_callback, 1)

    def _gps_callback(self, gps: Gps) -> None:
        message = Message()
        message.stamp = gps.header.stamp
        message.name = self.get_name()

        if gps.fix_type == Gps.FIX_3D:
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
