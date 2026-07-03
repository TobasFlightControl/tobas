# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

import numpy as np
from numpy import random as rnd
from typing import Callable


class NonlinearSimulator:
    """Simulator for nonlinear discrete-time state equations."""

    def __init__(
        self,
        x_dim: int,
        u_dim: int,
        y_dim: int,
        fx: Callable[[np.ndarray], np.ndarray],
        fu: Callable[[np.ndarray], np.ndarray],
        h: Callable[[np.ndarray], np.ndarray],
        Bv: np.ndarray,
        Q: np.ndarray,
        R: np.ndarray,
        init_x: np.ndarray = None,
    ) -> None:
        """
        Parameters
        ----------
        x_dim, u_dim, y_dim: int
            Dimensions of the state, control input, and output.
        fx, fu, h, Bv: np.ndarray
            Discrete-time state equations.
        Q: np.ndarray
            Covariance matrix of system noise.
        R: np.ndarray
            Covariance matrix of output noise.
        init_x: np.ndarray, default None
            Initial state.
        """

        self._x_dim = x_dim
        self._u_dim = u_dim
        self._v_dim = Bv.shape[1]
        self._y_dim = y_dim

        assert x_dim > 0
        assert u_dim >= 0, u_dim
        assert y_dim > 0
        assert Bv.shape == (self._x_dim, self._v_dim)
        assert Q.shape == (self._v_dim, self._v_dim)
        assert R.shape == (self._y_dim, self._y_dim)
        assert init_x is None or init_x.shape == (self._x_dim,)

        self._fx = fx
        self._fu = fu
        self._h = h
        self._Bv = np.array(Bv)
        self._Q = np.array(Q)
        self._R = np.array(R)
        self._x = np.array(init_x) if init_x is not None else np.zeros((self._x_dim,))

        self._v_mean = np.zeros((self._v_dim,))
        self._w_mean = np.zeros((self._y_dim,))

    def step(self, u: np.ndarray) -> np.ndarray:
        assert u.shape == (self._u_dim,)

        v = rnd.multivariate_normal(self._v_mean, self._Q)
        w = rnd.multivariate_normal(self._w_mean, self._R)

        self._x[:] = self._fx(self._x) + self._fu(u) + self._Bv @ v
        obs_y = self._h(self._x) + w

        return obs_y

    def reset(self, init_x: np.ndarray = None) -> None:
        assert init_x is None or init_x.shape == (self._x_dim,)

        self._x = init_x.copy() if init_x is not None else np.zeros((self._x_dim,))

    @property
    def x_true(self) -> np.ndarray:
        return self._x.copy()
