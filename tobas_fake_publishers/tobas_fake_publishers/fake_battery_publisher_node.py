import rclpy
from rclpy.node import Node

from tobas_msgs.msg import Battery


class FakeBatteryPublisherNode(Node):
    DEFAULT_SAMPLING_RATE = 100.0  # [Hz]
    DEFAULT_VOLTAGE = 14.8  # [V]
    DEFAULT_CURRENT = 20  # [A]

    def __init__(self) -> None:
        super().__init__("fake_battery_publisher")

        fs = self.declare_parameter("sampling_rate", self.DEFAULT_SAMPLING_RATE).get_parameter_value().double_value
        voltage = self.declare_parameter("voltage", self.DEFAULT_VOLTAGE).get_parameter_value().double_value
        current = self.declare_parameter("current", self.DEFAULT_CURRENT).get_parameter_value().double_value

        self._battery_msg = Battery()
        self._battery_msg.voltage = voltage
        self._battery_msg.current = current

        self._battery_pub = self.create_publisher(Battery, "battery", 1)
        self._timer = self.create_timer(1 / fs, self._timer_cb)

    def _timer_cb(self) -> None:
        self._battery_msg.header.stamp = self.get_clock().now().to_msg()
        self._battery_pub.publish(self._battery_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeBatteryPublisherNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
