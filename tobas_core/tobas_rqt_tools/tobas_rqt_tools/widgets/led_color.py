from __future__ import annotations
import numpy as np
from enum import Enum


class LEDColor(Enum):
    BLACK = np.array([0x00, 0x00, 0x00], dtype=np.uint8)
    WHITE = np.array([0xFF, 0xFF, 0xFF], dtype=np.uint8)
    BLUE = np.array([0x73, 0xCE, 0xF4], dtype=np.uint8)
    GREEN = np.array([0xAD, 0xFF, 0x2F], dtype=np.uint8)
    ORANGE = np.array([0xFF, 0xA5, 0x00], dtype=np.uint8)
    PURPLE = np.array([0xAF, 0x00, 0xFF], dtype=np.uint8)
    RED = np.array([0xF4, 0x37, 0x53], dtype=np.uint8)
    YELLOW = np.array([0xFF, 0xFF, 0x00], dtype=np.uint8)

    def __new__(cls, array: np.ndarray) -> LEDColor:
        obj = object.__new__(cls)
        obj._value_ = tuple(array)  # ndarrayをタプルに変換して不変にする
        return obj

    @property
    def array(self) -> np.ndarray:
        return np.array(self.value)  # タプルをndarrayに戻して返す
