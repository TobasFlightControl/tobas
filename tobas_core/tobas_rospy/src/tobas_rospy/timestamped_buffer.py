import rospy
from sortedcontainers import SortedDict


class TimestampedBuffer:

    def __init__(self, expiry_duration: rospy.Duration) -> None:
        self._expiry_duration = expiry_duration
        self._buffer = SortedDict()

    def items(self):
        return self._buffer.items()

    def add(self, cur_time: rospy.Time, x) -> None:
        self._buffer[cur_time] = x
        self._remove_expired_data(cur_time)

    def clear(self) -> None:
        self._buffer.clear()

    def _remove_expired_data(self, cur_time: rospy.Time) -> None:
        while len(self._buffer) > 0:
            time, _ = self._buffer.peekitem(0)
            if cur_time - time > self._expiry_duration:
                self._buffer.popitem(0)
            else:
                break
