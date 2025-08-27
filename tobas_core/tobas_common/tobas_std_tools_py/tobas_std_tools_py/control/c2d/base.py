import numpy as np
from typing import Callable
from abc import ABC, abstractmethod


class BaseC2D(ABC):
    def __init__(self, f: Callable[[np.ndarray], np.ndarray], dt: float) -> None:
        """
        C2Dのコンストラクタ

        Parameters
        ----------
        f : Callable[[np.ndarray[ndim=1]], np.ndarray[ndim=1]]
            1次微分方程式(xd = f(x))
        dt : float
            離散化幅
        """
        assert dt > 0.0

        self._f = f
        self._dt = dt

    @abstractmethod
    def __call__(self, x_cur: np.ndarray) -> np.ndarray:
        """
        微小時間後の状態を計算

        Parameters
        ----------
        x_cur : np.ndarray[ndim=1]
            現在の状態

        Returns
        -------
        x_next : np.ndarray[ndim=1]
            次の状態
        """
        raise NotImplementedError()
