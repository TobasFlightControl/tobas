import numpy as np
from typing import Tuple

from .base import Trajectory


class Cycloid3d(Trajectory):
    """z方向の誤差を含む3次元のサイクロイド"""

    def __init__(self, p0: np.ndarray, pf: np.ndarray, T: float, h: float, k: float = 5.0) -> None:
        """
        Cycloid3dオブジェクトを生成

        Parameters
        ----------
        p0 : np.ndarray[shape=(3,)]
            初期位置
        pf : np.ndarray[shape=(3,)]
            最終位置
        T : float
            時間長
        h : float
            pfからの最大高さ
        k : float, default 5.
            z方向のずれの修正強度
        """

        assert p0.shape == pf.shape == (3,)
        assert T > 0.0, f"T: {T}"
        assert h > 0.0, f"h: {h}"
        assert k >= 0.0, f"k: {k}"

        self._x0, self._y0, self._z0 = p0
        self._xf, self._yf, self._zf = pf
        self._x_diff, self._y_diff, self._z_diff = pf - p0
        self._T = T
        self._h = h
        self._k = k

    def __call__(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        assert 0.0 <= t < self._T

        pos = self._get_pos(t)
        vel = self._get_vel(t)
        acc = self._get_acc(t)

        return pos, vel, acc

    def get_duration(self) -> float:
        return self._T

    def _get_pos(self, t: float) -> np.ndarray:
        theta = 2 * np.pi * t / self._T
        tmp = (theta - np.sin(theta)) / (2 * np.pi)

        x = self._x0 + self._x_diff * tmp
        y = self._y0 + self._y_diff * tmp
        z = self._zf + self._h / 2 * (1 - np.cos(theta)) - self._z_diff * np.exp(-self._k * t / self._T)

        res = np.array([x, y, z])
        return res

    def _get_vel(self, t: float) -> np.ndarray:
        theta = 2 * np.pi * t / self._T
        tmp = (1 - np.cos(theta)) / self._T

        xd = self._x_diff * tmp
        yd = self._y_diff * tmp
        zd = (
            np.pi * self._h * np.sin(theta) / self._T
            + self._z_diff * self._k * np.exp(-self._k * t / self._T) / self._T
        )

        res = np.array([xd, yd, zd])
        return res

    def _get_acc(self, t: float) -> np.ndarray:
        theta = 2 * np.pi * t / self._T
        tmp = 2 * np.pi / self._T ** 2 * np.sin(theta)

        xdd = self._x_diff * tmp
        ydd = self._y_diff * tmp
        zdd = 2 * np.pi ** 2 * self._h / self._T ** 2 * np.cos(theta) - self._z_diff * (
            self._k / self._T
        ) ** 2 * np.exp(-self._k * t / self._T)

        res = np.array([xdd, ydd, zdd])
        return res
