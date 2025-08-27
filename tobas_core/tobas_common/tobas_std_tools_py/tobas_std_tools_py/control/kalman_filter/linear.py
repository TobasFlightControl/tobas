import numpy as np
import numpy.linalg as LA

from ...numpy_tools import is_symmetric, is_positive, is_semipositive
from ..observer.util import is_controllable, is_observable


class LinearKalmanFilter:
    """線形カルマンフィルタ(カルマンフィルタ入門, p.111)"""

    def __init__(
        self,
        A: np.ndarray,
        Bu: np.ndarray,
        Bv: np.ndarray,
        C: np.ndarray,
        Q: np.ndarray,
        R: np.ndarray,
        init_x: np.ndarray = None,
        init_P: np.ndarray = None,
    ) -> None:

        self._x_dim = A.shape[0]
        self._u_dim = Bu.shape[1]
        self._v_dim = Bv.shape[1]
        self._y_dim = C.shape[0]

        assert A.shape == (self._x_dim, self._x_dim)
        assert Bu.shape == (self._x_dim, self._u_dim)
        assert Bv.shape == (self._x_dim, self._v_dim)
        assert C.shape == (self._y_dim, self._x_dim)
        assert Q.shape == (self._v_dim, self._v_dim) and is_symmetric(Q) and is_semipositive(Q)
        assert R.shape == (self._y_dim, self._y_dim) and is_symmetric(R) and is_semipositive(R)
        assert init_x is None or init_x.shape == (self._x_dim,)
        assert init_P is None or (
            init_P.shape == (self._x_dim, self._x_dim) and is_symmetric(init_P) and is_positive(init_P)
        )

        assert self._u_dim == 0 or is_controllable(A, Bu)
        assert self._v_dim == 0 or is_controllable(A, Bv)
        assert self._y_dim == 0 or is_observable(A, C)

        self._A = np.array(A)
        self._Bu = np.array(Bu)
        self._Bv = np.array(Bv)
        self._C = np.array(C)
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
        x_post = self._x_prev + self._G @ (y - self._C @ self._x_prev)  # (6.58)
        P_post = (self._In - self._G @ self._C) @ self._P_prev  # (6.59)

        # 次の状態の事前推定
        self._x_prev = self._A @ x_post + self._Bu @ u  # (6.55)
        self._P_prev = self._A @ P_post @ self._A.T + self._Bv @ self._Q @ self._Bv.T  # (6.56)
        self._G = (self._P_prev @ self._C.T) @ LA.inv(self._C @ self._P_prev @ self._C.T + self._R)  # (6.57)

        return x_post

    def update_dynamics(self, A: np.ndarray, Bu: np.ndarray) -> None:
        assert A.shape == (self._x_dim, self._x_dim)
        assert Bu.shape == (self._x_dim, self._u_dim)

        self._A = A
        self._Bu = Bu
