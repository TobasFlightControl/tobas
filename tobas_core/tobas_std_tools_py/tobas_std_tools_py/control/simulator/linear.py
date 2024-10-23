import numpy as np
from numpy import random as rnd
from typing import List
from control import matlab


class LinearSimulator:
    """線形離散時間状態方程式のシミュレータ"""

    def __init__(
        self,
        A: np.ndarray,
        Bu: np.ndarray,
        Bv: np.ndarray,
        C: np.ndarray,
        Q: np.ndarray,
        R: np.ndarray,
        init_x: np.ndarray = None,
    ) -> None:
        """
        Parameters
        ----------
        A, Bu, Bv, C: np.ndarray
            離散時間状態方程式
        Q: np.ndarray
            システムノイズの共分散行列
        R: np.ndarray
            出力ノイズの共分散行列
        init_x: np.ndarray, default None
            初期状態
        """

        self._x_dim = A.shape[0]
        self._u_dim = Bu.shape[1]
        self._v_dim = Bv.shape[1]
        self._y_dim = C.shape[0]

        assert A.shape == (self._x_dim, self._x_dim)
        assert Bu.shape == (self._x_dim, self._u_dim)
        assert Bv.shape == (self._x_dim, self._v_dim)
        assert C.shape == (self._y_dim, self._x_dim)
        assert Q.shape == (self._v_dim, self._v_dim)
        assert R.shape == (self._y_dim, self._y_dim)
        assert init_x is None or init_x.shape == (self._x_dim,)

        self._A = np.array(A)
        self._Bu = np.array(Bu)
        self._Bv = np.array(Bv)
        self._C = np.array(C)
        self._Q = np.array(Q)
        self._R = np.array(R)
        self._x = np.array(init_x) if init_x is not None else np.zeros((self._x_dim,))

        self._v_mean = np.zeros((self._v_dim,))
        self._w_mean = np.zeros((self._y_dim,))

    def step(self, u: np.ndarray) -> np.ndarray:
        assert u.shape == (self._u_dim,)

        v = rnd.multivariate_normal(self._v_mean, self._Q)
        w = rnd.multivariate_normal(self._w_mean, self._R)

        self._x[:] = self._A @ self._x + self._Bu @ u + self._Bv @ v
        obs_y = self._C @ self._x + w

        return obs_y

    def reset(self, init_x: np.ndarray = None) -> None:
        assert init_x is None or init_x.shape == (self._x_dim,)

        self._x = init_x.copy() if init_x is not None else np.zeros((self._x_dim,))

    @property
    def x_true(self) -> np.ndarray:
        return self._x.copy()

    @classmethod
    def from_tf(cls, num: List[float], den: List[float], Ts: float, disc_method: str = "euler", **kwargs):
        sys_tf = matlab.tf(num, den)
        sys_cont = matlab.tf2ss(sys_tf)
        sys_disc = matlab.c2d(sys_cont, Ts=Ts, method=disc_method)

        A = np.array(sys_disc.A)
        Bu = np.array(sys_disc.B)
        C = np.array(sys_disc.C)

        x_dim = A.shape[0]
        v_dim = 0
        y_dim = C.shape[0]

        Bv = np.zeros((x_dim, v_dim))
        Q = (np.zeros((v_dim, v_dim)),)
        R = (np.zeros((y_dim, y_dim)),)

        return cls(A, Bu, Bv, C, Q, R, **kwargs)
