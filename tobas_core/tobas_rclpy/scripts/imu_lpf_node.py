import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from sensor_msgs.msg import Imu

from tobas_rclpy.conversions.np_msg import vectorMsgToNp, vectorNpToMsg
from tobas_rclpy.util import seconds_from_duration


class ImuLpf(Node):
    CUTOFF_FREQ = 30.0  # [Hz]

    def __init__(self) -> None:
        super().__init__("imu_lpf")

        self._tau = 0.5 / np.pi / self.CUTOFF_FREQ
        self._t_last = Time()
        self._acc = np.zeros((3,))
        self._gyro = np.zeros((3,))

        self._filterd_imu_pub = self.create_publisher(Imu, "filtered_imu", 1)
        self._imu_sub = self.create_subscription(Imu, "imu", self._imu_cb, 1)

        self.get_logger().info("Start to measure IMU.")

    def _imu_cb(self, imu: Imu) -> None:
        ts = max(seconds_from_duration(imu.header.stamp - self._t_last), 0.0)
        self._t_last = imu.header.stamp

        acc_raw = vectorMsgToNp(imu.linear_acceleration)
        gyro_raw = vectorMsgToNp(imu.angular_velocity)

        alpha = np.exp(-ts / self._tau)
        self._acc = alpha * self._acc + (1 - alpha) * acc_raw
        self._gyro = alpha * self._gyro + (1 - alpha) * gyro_raw

        imu.linear_acceleration = vectorNpToMsg(self._acc)
        imu.angular_velocity = vectorNpToMsg(self._gyro)

        self._filterd_imu_pub.publish(imu)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuLpf()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
