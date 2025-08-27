import numpy as np
import numpy.linalg as LA
from typing import Callable

from ...numpy_tools import is_symmetric, is_positive, is_semipositive


class ExtendedKalmanFilter:
    """拡張カルマンフィルタ(カルマンフィルタ入門, p.157)"""

    def __init__(
        self,
        x_dim: int,
        u_dim: int,
        y_dim: int,
        fx: Callable[[np.ndarray], np.ndarray],
        fu: Callable[[np.ndarray], np.ndarray],
        h: Callable[[np.ndarray], np.ndarray],
        dfx_dx: Callable[[np.ndarray], np.ndarray],
        dh_dx: Callable[[np.ndarray], np.ndarray],
        Bv: np.ndarray,
        Q: np.ndarray,
        R: np.ndarray,
        init_x: np.ndarray = None,
        init_P: np.ndarray = None,
    ) -> None:

        self._x_dim = x_dim
        self._u_dim = u_dim
        self._v_dim = Bv.shape[1]
        self._y_dim = y_dim

        assert x_dim > 0
        assert u_dim >= 0
        assert y_dim > 0
        assert Bv.shape == (self._x_dim, self._v_dim)
        assert Q.shape == (self._v_dim, self._v_dim) and is_symmetric(Q) and is_semipositive(Q)
        assert R.shape == (self._y_dim, self._y_dim) and is_symmetric(R) and is_semipositive(R)
        assert init_x is None or init_x.shape == (self._x_dim,)
        assert init_P is None or (
            init_P.shape == (self._x_dim, self._x_dim) and is_symmetric(init_P) and is_positive(init_P)
        )

        self._fx = fx
        self._fu = fu
        self._h = h
        self._dfx_dx = dfx_dx
        self._dh_dx = dh_dx
        self._Bv = np.array(Bv)
        self._Q = np.array(Q)
        self._R = np.array(R)
        self._x_prev = np.array(init_x) if init_x is not None else np.zeros((self._x_dim,))
        self._P_prev = np.array(init_P) if init_P is not None else np.identity(self._x_dim)
        self._G = np.zeros((self._x_dim, self._y_dim))
        self._In = np.identity(self._x_dim)

    def step(self, y: np.ndarray, u: np.ndarray) -> np.ndarray:
        assert y.shape == (self._y_dim,)
        assert u.shape == (self._u_dim,)

        # 現在の状態を推定
        C = self._dh_dx(self._x_prev)
        x_post = self._x_prev + self._G @ (y - self._h(self._x_prev))  # (7.21)
        P_post = (self._In - self._G @ C) @ self._P_prev  # (7.22)

        # 次の状態の事前推定
        A = self._dfx_dx(x_post)
        self._x_prev = self._fx(x_post) + self._fu(u)  # (7.17)
        self._P_prev = A @ P_post @ A.T + self._Bv @ self._Q @ self._Bv.T  # (7.19)
        self._G = (self._P_prev @ C.T) @ LA.inv(C @ self._P_prev @ C.T + self._R)  # (7.20)

        return x_post

    def update_dynamics(
        self,
        fx: Callable[[np.ndarray], np.ndarray],
        fu: Callable[[np.ndarray], np.ndarray],
        h: Callable[[np.ndarray], np.ndarray],
        dfx_dx: Callable[[np.ndarray], np.ndarray],
        dh_dx: Callable[[np.ndarray], np.ndarray],
    ) -> None:
        self._fx = fx
        self._fu = fu
        self._h = h
        self._dfx_dx = dfx_dx
        self._dh_dx = dh_dx
