import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Imu

from tobas_rclpy.conversions.np_msg import vectorMsgToNp


class MeasureImuVariance(Node):
    DATA_SIZE = 300

    def __init__(self) -> None:
        super().__init__("measure_imu_variance")

        self._acc_data = np.empty((self.DATA_SIZE, 3))
        self._gyro_data = np.empty((self.DATA_SIZE, 3))
        self._cnt = 0

        self._imu_sub = self.create_subscription(Imu, "imu", self._imu_cb, 1)

        self.get_logger().info("Start to measure IMU.")

    def _imu_cb(self, msg: Imu) -> None:
        self._acc_data[self._cnt, :] = vectorMsgToNp(msg.linear_acceleration)
        self._gyro_data[self._cnt, :] = vectorMsgToNp(msg.angular_velocity)
        self._cnt += 1

        if self._cnt == self.DATA_SIZE:
            mean_acc_var = self._acc_data.var(0)
            mean_gyro_var = self._gyro_data.var(0)
            self.get_logger().info(f"Acceleration std. dev [m/s^2]: {np.sqrt(mean_acc_var)}")
            self.get_logger().info(f"Gyro std. dev [rad/s]: {np.sqrt(mean_gyro_var)}")
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MeasureImuVariance()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
