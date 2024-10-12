import numpy as np
from numpy import random as rnd
from typing import Callable


class NonlinearSimulator:
    """非線形離散時間状態方程式のシミュレータ"""

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
            状態，制御入力，出力の次元
        fx, fu, h, Bv: np.ndarray
            離散時間状態方程式
        Q: np.ndarray
            システムノイズの共分散行列
        R: np.ndarray
            出力ノイズの共分散行列
        init_x: np.ndarray, default None
            初期状態
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
