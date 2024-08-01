import rclpy
from rclpy.duration import Duration

from tobas_tools_py.constants import Topic
from tobas_msgs.msg import Battery


class FakeBatteryPublisher:
    DEFAULT_SAMPLING_PERIOD = 0.01  # [m]
    DEFAULT_VOLTAGE = 14.8  # [V]
    DEFAULT_CURRENT = 20  # [A]

    def __init__(self) -> None:
        sampling_period = rclpy.get_param("~sampling_period", self.DEFAULT_SAMPLING_PERIOD)
        voltage = rclpy.get_param("~voltage", self.DEFAULT_VOLTAGE)
        current = rclpy.get_param("~current", self.DEFAULT_CURRENT)

        self._battery_msg = Battery()
        self._battery_msg.voltage = voltage
        self._battery_msg.current = current

        self._battery_pub = rclpy.Publisher(Topic.BATTERY, Battery, queue_size=1)
        self._timer = rclpy.Timer(Duration(sampling_period), self._timer_cb)

    def _timer_cb(self, event: rclpy.timer.TimerEvent) -> None:
        self._battery_msg.header.stamp = event.current_real
        self._battery_pub.publish(self._battery_msg)
