# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

import numpy as np
from typing import Tuple
from scipy.interpolate import UnivariateSpline

from .base import Trajectory


class UnivariateMultiSpline(Trajectory):
    """Multidimensional extension of `scipy.interpolate.UnivariateSpline`."""

    def __init__(self, waypoints: np.ndarray, times: np.ndarray, k: int = 3) -> None:
        """
        Create a `UnivariateMultiSpline` object.

        Parameters
        ----------
        waypoints : np.ndarray[shape=(n_splines, n_waypoints)]
            Waypoints.
        times : np.ndarray[shape=(n_waypoints,)]
            Times at each waypoint.
        k : float, default 3
            Spline degree.

        Note
        ----
        Only natural boundary conditions can be specified, where the accelerations at both ends are zero.
        Use `CubicMultiSpline` to specify velocity boundary conditions.
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
