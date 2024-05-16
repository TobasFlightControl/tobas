import rospy
from collections import deque
from threading import Lock
from typing import Deque, Tuple, Iterator, Any


class TimestampedBuffer:
    def __init__(self, expiry_duration: rospy.Duration) -> None:
        assert expiry_duration >= rospy.Duration(0)

        self._expiry_duration = expiry_duration
        self._que: Deque[Tuple[rospy.Time, Any]] = deque()
        self._index = 0
        self._lock = Lock()  # dequeにアクセスする部分をスレッドセーフにする

    def __iter__(self) -> Iterator[Tuple[rospy.Time, Any]]:
        return self

    def __next__(self) -> Tuple[rospy.Time, Any]:
        with self._lock:
            if self._index >= self.size():
                self._index = 0
                raise StopIteration()
            res = self._que[self._index]

        self._index += 1
        return res

    def add(self, cur_time: rospy.Time, data) -> None:
        # 要素を追加
        with self._lock:
            self._que.append((cur_time, data))

        # 期限切れの要素を削除
        self._remove_expired_data(cur_time)

    def clear(self) -> None:
        with self._lock:
            self._que.clear()

    def size(self) -> int:
        return len(self._que)

    def _remove_expired_data(self, cur_time: rospy.Time) -> None:
        # 繰り返し処理全体をロックすると遅延が大きくなるため，各繰り返し処理のみロックする．
        while True:
            with self._lock:
                # 要素が存在しなければ終了
                if self.size() == 0:
                    break

                # 最も古い要素を取り出す
                time, data = self._que.popleft()

                # 期限内の値が見つかったら，もとに戻して終了
                if cur_time - time < self._expiry_duration:
                    self._que.appendleft((time, data))
                    break
