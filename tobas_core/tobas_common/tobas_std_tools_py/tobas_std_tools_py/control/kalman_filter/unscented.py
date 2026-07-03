# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

import numpy as np
import scipy.linalg as SLA
from typing import Callable

from ...numpy_tools import is_symmetric, is_positive, is_semipositive


class UnscentedKalmanFilter:  # FIXME: `P` diverges and does not work well (2022/11/26).
    """Unscented Kalman filter (Fundamentals of Kalman Filter, p. 172)."""

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
        kappa: float = 0.0,
        init_x: np.ndarray = None,
        init_P: np.ndarray = None,
    ) -> None:
        """
        Constructor for `UnscentedKalmanFilter`.

        Parameters
        ----------
        x_dim : int
            Dimension of state `x`.
        u_dim : int
            Dimension of input `u`.
        y_dim : int
            Dimension of output `y`.
        fx : Callable[[np.ndarray], np.ndarray]
            Part of the discrete-time state equation that depends on state `x`.
        fu : Callable[[np.ndarray], np.ndarray]
            Part of the discrete-time state equation that depends on input `u`.
        h : Callable[[np.ndarray], np.ndarray]
            Output equation.
        Bv : np.ndarray
            Coefficient matrix of system noise in the discrete-time state equation.
        Q : np.ndarray
            Covariance matrix of system noise.
        R : np.ndarray
            Covariance matrix of observation noise.
        kappa : float, optional
            Scaling parameter (p. 164), by default 0.
        init_x : np.ndarray, optional
            Initial estimate of state `x`, by default `Zero(x_dim)`.
        init_P : np.ndarray, optional
            Initial estimate of the posterior covariance matrix,
            by default `Identity(x_dim)`.
        """
        self._x_dim = x_dim
        self._u_dim = u_dim
        self._v_dim = Bv.shape[1]
        self._y_dim = y_dim

        assert x_dim > 0
        assert u_dim >= 0
        assert y_dim > 0
        assert Bv.shape == (self._x_dim, self._v_dim)
        assert Q.shape == (self._v_dim, self._v_dim)
        assert is_symmetric(Q)
        assert is_semipositive(Q)
        assert R.shape == (self._y_dim, self._y_dim)
        assert is_symmetric(R)
        assert is_semipositive(R)
        assert kappa >= 0.0
        assert init_x is None or init_x.shape == (self._x_dim,)
        assert init_P is None or (
            init_P.shape == (self._x_dim, self._x_dim) and is_symmetric(init_P) and is_positive(init_P)
        )

        self._fx = fx
        self._fu = fu
        self._h = h
        self._Bv = np.array(Bv)
        self._Q = np.array(Q)
        self._R = np.array(R)
        self._kappa = kappa
        self._x_post = np.array(init_x) if init_x is not None else np.zeros((x_dim,))
        self._P_post = np.array(init_P) if init_P is not None else np.identity(x_dim)
        self._n_sample = 2 * x_dim + 1
        self._w = np.full((self._n_sample,), 1.0 / (2.0 * (x_dim + kappa)))
        self._w[0] = kappa / (x_dim + kappa)

    def step(self, y: np.ndarray, u: np.ndarray) -> np.ndarray:
        assert y.shape == (self._y_dim,), f"y.shape: {y.shape}, y_dim: {self._y_dim}"
        assert u.shape == (self._u_dim,), f"u.shape: {u.shape}, u_dim: {self._u_dim}"

        # Compute sigma points from the estimate at the previous time step.
        X_post = self._calc_X(self._x_post, self._P_post)  # (x_dim, n_sample)

        # Compute the current estimate for each sigma point.
        X_prev = self._dynamics(X_post, u)  # (x_dim, n_sample)

        # Compute prior estimates of current `x` and `P` as weighted sums of sigma points.
        x_prev = self._wsum_vec(X_prev)  # (x_dim,)
        P_prev = self._wsum_cov(X_prev, x_prev, X_prev, x_prev)
        P_prev += self._Bv @ self._Q @ self._Bv.T

        # Recompute sigma points using the prior state value.
        X_prev = self._calc_X(x_prev, P_prev)

        # Update output sigma points.
        Y_prev = self._output(X_prev)

        # Compute prior estimates of current `y`, `Pyy`, and `Pxy` as weighted sums of sigma points.
        y_prev = self._wsum_vec(Y_prev)
        Pyy_prev = self._wsum_cov(Y_prev, y_prev, Y_prev, y_prev)
        Pxy_prev = self._wsum_cov(X_prev, x_prev, Y_prev, y_prev)

        # Compute the Kalman gain.
        G = Pxy_prev @ SLA.inv(Pyy_prev + self._R)

        # Compute posterior estimates of `x` and `P`.
        self._x_post = x_prev + G @ (y - y_prev)
        self._P_post = P_prev - G @ Pxy_prev.T

        return self._x_post.copy()

    def _dynamics(self, X: np.ndarray, u: np.ndarray) -> np.ndarray:
        assert X.shape == (self._x_dim, self._n_sample)
        assert u.shape == (self._u_dim,)

        fu = self._fu(u)
        res = np.empty((self._x_dim, self._n_sample))
        for i in range(0, self._n_sample):
            res[:, i] = self._fx(X[:, i]) + fu

        return res

    def _output(self, X: np.ndarray) -> np.ndarray:
        assert X.shape == (self._x_dim, self._n_sample)

        res = np.empty((self._y_dim, self._n_sample))
        for i in range(0, self._n_sample):
            res[:, i] = self._h(X[:, i])

        return res

    def _calc_X(self, x: np.ndarray, P: np.ndarray) -> np.ndarray:
        assert x.shape == (self._x_dim,), f"{x.shape} != {(self._x_dim,)}"
        assert P.shape == (
            self._x_dim,
            self._x_dim,
        ), f"{P.shape} != {(self._x_dim, self._x_dim)}"

        sqrt_P = SLA.sqrtm(P)  # Square root matrix of `P`.
        c = np.sqrt(self._x_dim + self._kappa)

        # print(P)
        # print(is_positive(P))
        # print(np.linalg.matrix_rank(sqrt_P))

        res = np.empty((self._x_dim, self._n_sample))
        res[:, 0] = x
        for i in range(0, self._x_dim):
            res[:, i] = x + c * sqrt_P[:, i]
            res[:, self._x_dim + i] = x - c * sqrt_P[:, i]

        return res

    def _wsum_vec(self, A: np.ndarray) -> np.ndarray:
        assert A.ndim == 2 and A.shape[1] == self._n_sample
        return np.sum(self._w[np.newaxis, :] * A, axis=1)

    def _wsum_cov(self, A: np.ndarray, a_mean: np.ndarray, B: np.ndarray, b_mean: np.ndarray) -> np.ndarray:
        a_dim = A.shape[0]
        b_dim = B.shape[0]

        assert A.shape == (a_dim, self._n_sample)
        assert a_mean.shape == (a_dim,)
        assert B.shape == (b_dim, self._n_sample)
        assert b_mean.shape == (b_dim,)

        res = np.zeros((a_dim, b_dim))
        for i in range(0, self._n_sample):
            a_diff = (A[:, i] - a_mean).reshape(-1, 1)
            b_diff = (B[:, i] - b_mean).reshape(-1, 1)
            res += self._w[i] * (a_diff @ b_diff.T)

        return res
