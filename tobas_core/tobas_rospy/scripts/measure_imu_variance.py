#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Imu

from tobas_rospy.utils import init_node
from tobas_rospy.conversions.np_msg import vectorMsgToNp


class MeasureImuVariance:
    DATA_SIZE = 300

    def __init__(self) -> None:
        self._acc_data = np.empty((self.DATA_SIZE, 3))
        self._gyro_data = np.empty((self.DATA_SIZE, 3))
        self._cnt = 0

        self._imu_sub = rospy.Subscriber("imu", Imu, self._imu_cb)

        rospy.loginfo("Start to measure IMU.")

    def _imu_cb(self, msg: Imu) -> None:
        rospy.loginfo_once("First IMU message is received.")

        self._acc_data[self._cnt, :] = vectorMsgToNp(msg.linear_acceleration)
        self._gyro_data[self._cnt, :] = vectorMsgToNp(msg.angular_velocity)
        self._cnt += 1

        if self._cnt == self.DATA_SIZE:
            mean_acc_var = self._acc_data.var(0)
            mean_gyro_var = self._gyro_data.var(0)
            rospy.loginfo(f"Acceleration std. dev [m/s^2]: {np.sqrt(mean_acc_var)}")
            rospy.loginfo(f"Gyro std. dev [rad/s]: {np.sqrt(mean_gyro_var)}")
            rospy.signal_shutdown("Finished")


if __name__ == "__main__":
    init_node()
    node = MeasureImuVariance()
    rospy.spin()
