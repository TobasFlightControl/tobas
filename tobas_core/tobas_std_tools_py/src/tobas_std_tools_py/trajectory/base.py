from typing import Tuple
from abc import ABC, abstractmethod


class Trajectory(ABC):
    """多次元軌道生成器の基底クラス"""

    @abstractmethod
    def __call__(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        位置，速度，加速度を生成する

        Parameters
        ----------
        t : float
            時刻

        Returns
        -------
        pos : np.ndarray[ndim=1]
            位置
        vel : np.ndarray[ndim=1]
            速度
        acc : np.ndarray[ndim=1]
            加速度
        """
        raise NotImplementedError()

    @abstractmethod
    def get_duration(self) -> float:
        """軌跡の時間長を返す"""
        raise NotImplementedError()
