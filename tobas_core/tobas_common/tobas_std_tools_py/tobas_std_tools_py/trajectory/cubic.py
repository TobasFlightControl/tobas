# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

import numpy as np
from typing import Tuple
from scipy.interpolate import CubicSpline

from .base import Trajectory


class CubicMultiSpline(Trajectory):
    """Multidimensional extension of `scipy.interpolate.CubicSpline`."""

    def __init__(
        self,
        waypoints: np.ndarray,
        times: np.ndarray,
        bc_type: int = 1,
        end1: np.ndarray = None,
        end2: np.ndarray = None,
    ) -> None:
        """
        Create a `CubicMultiSpline` object.

        Parameters
        ----------
        waypoints : np.ndarray[shape=(n_splines, n_waypoints)]
            Waypoints.
        times : np.ndarray[shape=(n_waypoints,)]
            Times at each waypoint.
        bc_type : int, default 2
            Boundary condition type.
            Specify first- or second-order derivatives.
        end1 : np.ndarray[shape=(n_splines,)], default None
            Boundary condition at the start point.
        end2 : np.ndarray[shape=(n_splines,)], default None
            Boundary condition at the end point.
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
