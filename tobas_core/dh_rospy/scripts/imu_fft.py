#!/usr/bin/env python3

import os.path as osp
import numpy as np
from numpy import fft
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import Imu

from dh_rospy.conversions.np_msg import vector3_msg_to_np


class ImuFft:
    DEFAULT_DATA_SIZE = 5000

    def __init__(self) -> None:
        self._data_size = rospy.get_param("~data_size", self.DEFAULT_DATA_SIZE)
        assert self._data_size > 0

        self._acc_data = np.empty((self._data_size, 3))
        self._gyro_data = np.empty((self._data_size, 3))
        self._cnt = 0
        self._start_time = rospy.Time()

        self._imu_sub = rospy.Subscriber("imu", Imu, self._imu_cb)

        rospy.loginfo("Start to measure IMU.")

    def _imu_cb(self, msg: Imu) -> None:
        if self._cnt == 0:
            rospy.loginfo("First IMU message is received.")
            self._start_time = msg.header.stamp

        self._acc_data[self._cnt, :] = vector3_msg_to_np(msg.linear_acceleration)
        self._gyro_data[self._cnt, :] = vector3_msg_to_np(msg.angular_velocity)
        self._cnt += 1

        if self._cnt == self._data_size:
            self._imu_sub.unregister()

            # 平均を除いてフーリエ変換 (データ数による正規化を忘れずに)
            acc_data = self._acc_data - self._acc_data.mean(axis=0, keepdims=True)
            gyro_data = self._gyro_data - self._gyro_data.mean(axis=0, keepdims=True)
            acc_fft = fft.fftn(acc_data, axes=(0,)) / self._data_size * 2  # [m/s^2]
            gyro_fft = fft.fftn(gyro_data, axes=(0,)) / self._data_size * 2  # [rad/s]

            end_time = msg.header.stamp
            Ts = (end_time - self._start_time).to_sec() / self._data_size
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

            rospy.signal_shutdown("Finished")


if __name__ == "__main__":
    node_name = osp.splitext(osp.basename(__file__))[0]
    rospy.init_node(node_name)
    node = ImuFft()
    rospy.spin()
