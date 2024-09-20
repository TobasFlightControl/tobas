import rclpy
from rclpy.node import Node

from tobas_msgs.msg import Gps


class FakeGNSSPublisherNode(Node):
    DEFAULT_SAMPLING_RATE = 5.0  # [Hz]
    DEFAULT_POS_STDDEV = 3.0  # [m]
    DEFAULT_VEL_STDDEV = 0.3  # [m/s]

    def __init__(self) -> None:
        super().__init__("fake_gnss_publisher")

        fs = self.declare_parameter("sampling_rate", self.DEFAULT_SAMPLING_RATE).get_parameter_value().double_value
        pos_stddev = (
            self.declare_parameter("position_stddev", self.DEFAULT_POS_STDDEV).get_parameter_value().double_value
        )
        vel_stddev = (
            self.declare_parameter("velocity_stddev", self.DEFAULT_VEL_STDDEV).get_parameter_value().double_value
        )

        pos_var = pos_stddev**2
        vel_var = vel_stddev**2

        self._gnss_msg = Gps()
        self._gnss_msg.fix_type = Gps.FIX_3D
        self._gnss_msg.latitude = 0.0
        self._gnss_msg.longitude = 0.0
        self._gnss_msg.altitude = 0.0
        self._gnss_msg.ground_speed.x = 0.0
        self._gnss_msg.ground_speed.y = 0.0
        self._gnss_msg.ground_speed.z = 0.0
        self._gnss_msg.position_covariance.data = [pos_var, 0, 0, 0, pos_var, 0, 0, 0, pos_var]
        self._gnss_msg.velocity_covariance.data = [vel_var, 0, 0, 0, vel_var, 0, 0, 0, vel_var]

        self._gnss_pub = self.create_publisher(Gps, "gps", 1)
        self._timer = self.create_timer(1 / fs, self._timer_cb)

    def _timer_cb(self) -> None:
        self._gnss_msg.header.stamp = self.get_clock().now().to_msg()
        self._gnss_pub.publish(self._gnss_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeGNSSPublisherNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
