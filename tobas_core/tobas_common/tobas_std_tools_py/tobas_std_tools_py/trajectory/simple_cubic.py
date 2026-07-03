# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

import numpy as np
from typing import Tuple

from .base import Trajectory


class SimpleCubicSpline(Trajectory):
    """Cubic polynomial trajectory generation (Introduction to ROBOTICS, p. 192)."""

    def __init__(self, x0: np.ndarray, xf: np.ndarray, T: float) -> None:
        """
        Create a `SimpleCubicSpline` object.

        Parameters
        ----------
        x0 : np.ndarray[ndim=1]
            Initial position.
        xf : np.ndarray[ndim=1]
            Final position.
        T : float
            Duration.
        """

        assert x0.ndim == xf.ndim == 1
        assert x0.shape == xf.shape
        assert T > 0.0

        self._T = T

        self._a0 = x0.copy()
        self._a1 = np.zeros_like(x0)
        self._a2 = (3 / T**2) * (xf - x0)
        self._a3 = (-2 / T**3) * (xf - x0)

    def __call__(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        pos = self._a0 + self._a1 * t + self._a2 * t**2 + self._a3 * t**3
        vel = self._a1 + 2 * self._a2 * t + 3 * self._a3 * t**2
        acc = 2 * self._a2 + 6 * self._a3 * t
        return pos, vel, acc

    def get_duration(self) -> float:
        return self._T
