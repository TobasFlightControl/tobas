import numpy as np
from typing import Tuple
from scipy.interpolate import CubicSpline

from .base import Trajectory


class CubicMultiSpline(Trajectory):
    """scipy.interpolate.CubicSplineの多次元拡張"""

    def __init__(
        self,
        waypoints: np.ndarray,
        times: np.ndarray,
        bc_type: int = 1,
        end1: np.ndarray = None,
        end2: np.ndarray = None,
    ) -> None:
        """
        CubicMultiSplineオブジェクトを生成

        Parameters
        ----------
        waypoints : np.ndarray[shape=(n_splines, n_waypoints)]
            初期位置
        times : np.ndarray[shape=(n_waypoints,)]
            各waypointでの時刻
        bc_type : int, default 2
            境界条件のタイプ．1階微分または2階微分
        end1 : np.ndarray[shape=(n_splines,)], default None
            始点の境界条件
        end2 : np.ndarray[shape=(n_splines,)], default None
            終点の境界条件
        """

        self._n_splines, self._n_waypoints = waypoints.shape
        self._duration = times[-1]

        assert waypoints.shape == (self._n_splines, self._n_waypoints)
        assert times.shape == (self._n_waypoints,)
        assert bc_type in {1, 2}

        if end1 is None:
            end1 = np.zeros((self._n_splines,))
        else:
            assert end1.shape == (self._n_splines,)
        if end2 is None:
            end2 = np.zeros((self._n_splines,))
        else:
            assert end2.shape == (self._n_splines,)

        self._pos_funcs = []
        self._vel_funcs = []
        self._acc_funcs = []
        for i, poss in enumerate(waypoints):
            spline = CubicSpline(times, poss, bc_type=((bc_type, end1[i]), (bc_type, end2[i])))
            self._pos_funcs.append(spline)
            self._vel_funcs.append(spline.derivative(nu=1))
            self._acc_funcs.append(spline.derivative(nu=2))

    def __call__(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        assert 0.0 <= t <= self._duration, f"t: {t}, duration: {self._duration}"

        pos = []
        vel = []
        acc = []
        for pos_func, vel_func, acc_func in zip(self._pos_funcs, self._vel_funcs, self._acc_funcs):
            pos.append(pos_func(t))
            vel.append(vel_func(t))
            acc.append(acc_func(t))

        return np.array(pos), np.array(vel), np.array(acc)

    def get_duration(self) -> float:
        return self._duration
