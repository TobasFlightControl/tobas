# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

import numpy as np
from typing import Callable
from abc import ABC, abstractmethod


class BaseC2D(ABC):
    def __init__(self, f: Callable[[np.ndarray], np.ndarray], dt: float) -> None:
        """
        Constructor for `C2D`.

        Parameters
        ----------
        f : Callable[[np.ndarray[ndim=1]], np.ndarray[ndim=1]]
            First-order differential equation, `xd = f(x)`.
        dt : float
            Discretization interval.
        """
        assert dt > 0.0

        self._f = f
        self._dt = dt

    @abstractmethod
    def __call__(self, x_cur: np.ndarray) -> np.ndarray:
        """
        Compute the state after a small time interval.

        Parameters
        ----------
        x_cur : np.ndarray[ndim=1]
            Current state.

        Returns
        -------
        x_next : np.ndarray[ndim=1]
            Next state.
        """
        raise NotImplementedError()
