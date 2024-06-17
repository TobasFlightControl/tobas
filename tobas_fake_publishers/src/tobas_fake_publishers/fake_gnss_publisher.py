import rospy

from tobas_tools_py.constants import Topic
from tobas_msgs.msg import Gps


class FakeGNSSPublisher:
    DEFAULT_SAMPLING_PERIOD = 0.2  # [s]
    DEFAULT_POS_STDDEV = 3.0  # [m]
    DEFAULT_VEL_STDDEV = 0.3  # [m/s]

    def __init__(self) -> None:
        sampling_period = rospy.get_param("~sampling_period", self.DEFAULT_SAMPLING_PERIOD)
        pos_stddev = rospy.get_param("~position_stddev", self.DEFAULT_POS_STDDEV)
        vel_stddev = rospy.get_param("~velocity_stddev", self.DEFAULT_VEL_STDDEV)

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
        self._gnss_msg.position_covariance = [pos_var, 0, 0, 0, pos_var, 0, 0, 0, pos_var]
        self._gnss_msg.velocity_covariance = [vel_var, 0, 0, 0, vel_var, 0, 0, 0, vel_var]

        self._gnss_pub = rospy.Publisher(Topic.GNSS, Gps, queue_size=1)
        self._timer = rospy.Timer(rospy.Duration(sampling_period), self._timer_cb)

    def _timer_cb(self, event: rospy.timer.TimerEvent) -> None:
        self._gnss_msg.header.stamp = event.current_real
        self._gnss_pub.publish(self._gnss_msg)
