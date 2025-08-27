import numpy as np
from typing import Callable

from .base import BaseC2D


class C2D_Euler(BaseC2D):
    """オイラー法による一次微分方程式の離散化"""

    def __init__(self, f: Callable[[np.ndarray], np.ndarray], dt: float) -> None:
        super().__init__(f, dt)

    def __call__(self, x_cur: np.ndarray) -> np.ndarray:
        assert x_cur.ndim == 1

        x_next = x_cur + self._f(x_cur) * self._dt
        return x_next
