from collections import deque
from typing import List, Iterable

from dh_function.basic import is_unique


class MSequenceGenerator:
    def __init__(self, num_registers: int, taps: List[int] = None, initial_state: List[bool] = None) -> None:
        assert num_registers >= 2

        if taps is None:
            # 原始多項式に基づいて最大周期を得るためのタップ位置を決定
            # cf. http://www.finetune.co.jp/~lyuka/technote/lfsr/lfsr.html
            if num_registers == 2:
                self._taps = [0, 1]
            elif num_registers == 3:
                self._taps = [0, 2]
            elif num_registers == 4:
                self._taps = [0, 3]
            elif num_registers == 5:
                self._taps = [1, 4]
            elif num_registers == 6:
                self._taps = [0, 5]
            elif num_registers == 7:
                self._taps = [0, 6]
            elif num_registers == 8:
                self._taps = [1, 2, 3, 7]
            elif num_registers == 9:
                self._taps = [3, 8]
            elif num_registers == 10:
                self._taps = [2, 9]
            else:
                raise NotImplementedError()
        else:
            assert len(taps) >= 2
            assert is_unique(taps)
            assert min(taps) >= 0
            assert max(taps) == num_registers - 1
            self._taps = taps

        if initial_state is None:
            self._initial_state = [False] * num_registers
            self._initial_state[-1] = True
        else:
            assert len(initial_state) == num_registers
            assert min(initial_state) >= 0
            assert max(initial_state) <= 1
            assert sum(initial_state) > 0
            self._initial_state = initial_state

        self._register = deque(self._initial_state)
        self._length = 2 ** len(self._initial_state) - 1
        self._count = 0

    def __iter__(self) -> Iterable:
        return self

    def __next__(self) -> int:
        if self._count == self._length:
            self._register = deque(self._initial_state)
            self._count = 0
            raise StopIteration()

        # フィードバックビットを計算
        feedback = sum([self._register[i] for i in self._taps]) % 2

        # レジスタを1ビットずつシフト
        self._register.pop()
        self._register.appendleft(feedback)
        self._count += 1

        return self._register[-1] * 2 - 1  # [0, 1] -> [-1, 1]

    def length(self) -> int:
        """1周期の長さ．"""
        return self._length
