import numpy as np
from typing import Callable

from .base import BaseC2D


class C2D_RK4(BaseC2D):
    """4次のルンゲクッタ法による一次微分方程式の離散化"""

    def __init__(self, f: Callable[[np.ndarray], np.ndarray], dt: float) -> None:
        super().__init__(f, dt)

    def __call__(self, x_cur: np.ndarray) -> np.ndarray:
        assert x_cur.ndim == 1

        k1 = self._f(x_cur)
        k2 = self._f(x_cur + self._dt / 2 * k1)
        k3 = self._f(x_cur + self._dt / 2 * k2)
        k4 = self._f(x_cur + self._dt * k3)

        x_next = x_cur + (self._dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
        return x_next
