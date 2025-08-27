import numpy as np
import numpy.linalg as LA
from control import matlab
from rich import print as rprint

from .basic import BaseObserver


class MeasurableDisturbanceObserver(BaseObserver):
    """状態方程式に測定可能な一定値外乱が含まれる場合のオブザーバ(p.179)"""

    def __init__(
        self,
        A: np.ndarray,
        B: np.ndarray,
        Bd: np.ndarray,
        Cy: np.ndarray,
        Cz: np.ndarray,
        Ts: float,
        Lx: np.ndarray = None,
        init_x: np.ndarray = None,
        disc_method: str = "euler",
    ) -> None:
        """
        Parameters
        ----------
        A, B, Bd, Cy, Cz: np.ndarray
            連続時間状態方程式の係数行列
        Ts: float
            サンプリング時間
        Lx: np.ndarray, default None
            オブザーバのゲイン行列のxの部分
        init_x: np.ndarray, default None
            初期状態
        disc_method: str, default 'euler'
            離散化手法

        Returns
        ----------
        None

        Note
        ----------
        - Lxは離散時間にそのまま適用されることに注意
        """

        super().__init__()

        self._x_dim = A.shape[0]
        self._u_dim = B.shape[1]
        self._d_dim = Bd.shape[0]
        self._y_dim = Cy.shape[0]
        self._z_dim = Cz.shape[0]

        if Lx is None:
            Lx = np.zeros((self._x_dim, self._y_dim))

        assert A.ndim == 2 and A.shape == (self._x_dim, self._x_dim)
        assert B.ndim == 2 and B.shape == (self._x_dim, self._u_dim)
        assert Bd.ndim == 2 and Bd.shape == (self._x_dim, self._d_dim)
        assert Cy.ndim == 2 and Cy.shape == (self._y_dim, self._x_dim)
        assert Cz.ndim == 2 and Cz.shape == (self._z_dim, self._x_dim)
        assert Lx.ndim == 2 and Lx.shape == (self._x_dim, self._d_dim)
        assert Ts > 0.0

        xi_dim = self._x_dim + self._d_dim
        A_tilda_cont = np.zeros((xi_dim, xi_dim))
        A_tilda_cont[: self._x_dim, : self._x_dim] = A
        A_tilda_cont[: self._x_dim, -self._d_dim :] = Bd
        B_tilda_cont = np.r_[B, np.zeros((self._d_dim, self._u_dim))]
        C_tilda_cont = np.zeros((self._y_dim + self._d_dim, self._x_dim + self._d_dim))
        C_tilda_cont[: self._y_dim, : self._x_dim] = Cy
        C_tilda_cont[-self._d_dim :, -self._d_dim :] = np.identity(self._d_dim)

        cont_sys = matlab.ss(
            A_tilda_cont,
            B_tilda_cont,
            C_tilda_cont,
            np.zeros((C_tilda_cont.shape[0], B_tilda_cont.shape[1])),
        )
        disc_sys = matlab.c2d(cont_sys, Ts, method=disc_method)
        A_tilda_disc = np.array(disc_sys.A)
        B_tilda_disc = np.array(disc_sys.B)

        # ちゃんと一定値外乱モデルになっていることを確認
        A21 = A_tilda_disc[-self._d_dim :, : self._x_dim]
        A22 = A_tilda_disc[-self._d_dim :, -self._d_dim :]
        B2 = B_tilda_disc[-self._d_dim :, :]
        assert np.allclose(A21, 0.0)
        assert np.allclose(A22, np.identity(self._d_dim))
        assert np.allclose(B2, 0.0)

        self._A = A_tilda_disc[: self._x_dim, : self._x_dim]
        self._B = B_tilda_disc[: self._x_dim, :]
        self._Bd = A_tilda_disc[: self._x_dim, -self._d_dim :]
        self._Cy = Cy.copy()
        self._Cz = Cz.copy()
        self._L = Lx.copy()
        self._Ts = Ts

        self._K = self._A - self._L @ self._Cy
        self._x = init_x.copy() if init_x is not None else np.zeros((self._x_dim,))
        self._y = self.Cy @ self._x
        self._z = self.Cz @ self._x

        # 収束性の確認: Kの固有値が単位円の外にある場合は収束性が保証されない
        disc_eig, _ = LA.eig(self._K)
        max_abs_disc_eig = max(abs(disc_eig))
        if max_abs_disc_eig > 1.0:
            rprint("[yellow]Warning: The system is unstable.[/yellow]")
            rprint(f"[yellow]eigenvalues of K: {disc_eig}[/yellow]")

    def step(self, y: np.ndarray, d: np.ndarray, u: np.ndarray) -> None:
        assert y.shape == (self._y_dim,)
        assert d.shape == (self._d_dim,)
        assert u.shape == (self._u_dim,)

        self._x[:] = self._K @ self._x + self._B @ u + self._Bd @ d + self._L @ y
        self._y[:] = self._Cy @ self._x
        self._z[:] = self._Cz @ self._x

    @property
    def Bd(self) -> np.ndarray:
        return self._Bd

    @property
    def d_dim(self) -> int:
        return self._d_dim
