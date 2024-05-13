import numpy as np
from typing import Tuple
from scipy.interpolate import UnivariateSpline

from .base import Trajectory


class UnivariateMultiSpline(Trajectory):
    """scipy.interpolate.UnivariateSplineの多次元拡張"""

    def __init__(self, waypoints: np.ndarray, times: np.ndarray, k: int = 3) -> None:
        """
        UnivariateMultiSplineオブジェクトを生成

        Parameters
        ----------
        waypoints : np.ndarray[shape=(n_splines, n_waypoints)]
            初期位置
        times : np.ndarray[shape=(n_waypoints,)]
            各waypointでの時刻
        k : float, default 3
            スプラインの次元

        Note
        ----
        境界条件は自然境界条件(両端の加速度が0)しか指定できない．
        速度の境界条件にしたければCubicMultiSplineを使う．
        """

        self._n_splines, self._n_waypoints = waypoints.shape
        self._duration = times[-1]

        assert waypoints.shape == (self._n_splines, self._n_waypoints)
        assert times.shape == (self._n_waypoints,)
        assert k > 0

        self._pos_funcs = []
        self._vel_funcs = []
        self._acc_funcs = []
        for poss in waypoints:
            spline = UnivariateSpline(times, poss, k=k, s=0.0, ext="raise")
            self._pos_funcs.append(spline)
            self._vel_funcs.append(spline.derivative(n=1))
            if k >= 2:
                self._acc_funcs.append(spline.derivative(n=2))
            else:
                self._acc_funcs.append(lambda x: 0.0)

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
