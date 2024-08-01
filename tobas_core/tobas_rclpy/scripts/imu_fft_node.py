import numpy as np
from numpy import fft
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu

from tobas_rclpy.util import seconds_from_duration
from tobas_rclpy.conversions.np_msg import vectorMsgToNp


class ImuFft(Node):
    DEFAULT_DATA_SIZE = 5000

    def __init__(self) -> None:
        super().__init__("imu_fft")

        self.declare_parameter("data_size", self.DEFAULT_DATA_SIZE)
        self._data_size = self.get_parameter("data_size").get_parameter_value().integer_value
        assert self._data_size > 0

        self._acc_data = np.empty((self._data_size, 3))
        self._gyro_data = np.empty((self._data_size, 3))
        self._cnt = 0
        self._start_time = Time()

        self._imu_sub = self.create_subscription(Imu, "imu", self._imu_cb, 1)

        self.get_logger().info("Start to measure IMU.")

    def _imu_cb(self, msg: Imu) -> None:
        if self._cnt == 0:
            self.get_logger().info("First IMU message is received.")
            self._start_time = msg.header.stamp

        self._acc_data[self._cnt, :] = vectorMsgToNp(msg.linear_acceleration)
        self._gyro_data[self._cnt, :] = vectorMsgToNp(msg.angular_velocity)
        self._cnt += 1

        if self._cnt == self._data_size:
            self.destroy_subscription(self._imu_sub)

            # 平均を除いてフーリエ変換 (データ数による正規化を忘れずに)
            acc_data = self._acc_data - self._acc_data.mean(axis=0, keepdims=True)
            gyro_data = self._gyro_data - self._gyro_data.mean(axis=0, keepdims=True)
            acc_fft = fft.fftn(acc_data, axes=(0,)) / self._data_size * 2  # [m/s^2]
            gyro_fft = fft.fftn(gyro_data, axes=(0,)) / self._data_size * 2  # [rad/s]

            end_time = msg.header.stamp
            Ts = seconds_from_duration(end_time - self._start_time) / self._data_size
            fs = 1 / Ts
            freq = fft.fftfreq(self._data_size, Ts)

            plt.figure(figsize=(15, 9))

            for i, axis in enumerate(["X", "Y", "Z"]):
                plt.subplot(6, 1, i + 1)
                plt.plot(freq, np.abs(acc_fft[:, i]))
                plt.title(f"Frequency Spectrum of Accel {axis} data")
                plt.xlabel("Frequency (Hz)")
                plt.ylabel("Amplitude [m/s^2]")
                plt.xlim([0, fs / 2])

            for i, axis in enumerate(["X", "Y", "Z"]):
                plt.subplot(6, 1, i + 4)
                plt.plot(freq, np.abs(gyro_fft[:, i]))
                plt.title(f"Frequency Spectrum of Gyro {axis} data")
                plt.xlabel("Frequency (Hz)")
                plt.ylabel("Amplitude [rad/s]")
                plt.xlim([0, fs / 2])

            plt.tight_layout()
            plt.show()

            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuFft()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
