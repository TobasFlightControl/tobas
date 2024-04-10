import rospy
from collections import deque
from typing import Deque, Tuple, Iterator, Any


class TimestampedBuffer:

    def __init__(self, expiry_duration: rospy.Duration) -> None:
        self._expiry_duration = expiry_duration
        self._que: Deque[rospy.Time, Any] = deque()

    def __iter__(self) -> Iterator[Tuple[rospy.Time, Any]]:
        return iter(self._que)

    def add(self, cur_time: rospy.Time, data) -> None:
        # FIXME: ただ後ろに追加するだけだと時刻の順序が保証されない．std::mapのような赤黒木を使うのが理想．
        self._que.append((cur_time, data))
        self._remove_expired_data(cur_time)

    def clear(self) -> None:
        self._que.clear()

    def _remove_expired_data(self, cur_time: rospy.Time) -> None:
        while len(self._que) > 0:
            time, data = self._que.popleft()

            # 期限内の値が見つかったら，もとに戻して終了
            if cur_time - time < self._expiry_duration:
                self._que.appendleft((time, data))
                break
