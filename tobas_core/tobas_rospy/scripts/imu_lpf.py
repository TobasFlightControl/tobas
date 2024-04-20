#!/usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import Imu

from tobas_rospy.conversions.np_msg import *
from tobas_rospy.utils import init_node


class ImuLpf:
    CUTOFF_FREQ = 30.0  # [Hz]

    def __init__(self) -> None:
        self._tau = 0.5 / np.pi / self.CUTOFF_FREQ
        self._t_last = rospy.Time(0)
        self._acc = np.zeros((3,))
        self._gyro = np.zeros((3,))

        self._filterd_imu_pub = rospy.Publisher("filtered_imu", Imu, queue_size=1)
        self._imu_sub = rospy.Subscriber("imu", Imu, self._imu_cb)

        rospy.loginfo("Start to measure IMU.")

    def _imu_cb(self, imu: Imu) -> None:
        rospy.loginfo_once("First IMU message is received.")

        ts = max((imu.header.stamp - self._t_last).to_sec(), 0.0)
        self._t_last = imu.header.stamp

        acc_raw = vector3_msg_to_np(imu.linear_acceleration)
        gyro_raw = vector3_msg_to_np(imu.angular_velocity)

        alpha = np.exp(-ts / self._tau)
        self._acc = alpha * self._acc + (1 - alpha) * acc_raw
        self._gyro = alpha * self._gyro + (1 - alpha) * gyro_raw

        imu.linear_acceleration = vector3_np_to_msg(self._acc)
        imu.angular_velocity = vector3_np_to_msg(self._gyro)

        self._filterd_imu_pub.publish(imu)


if __name__ == "__main__":
    init_node()
    node = ImuLpf()
    rospy.spin()
