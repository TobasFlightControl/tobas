import numpy as np
import numpy.linalg as LA
from control import matlab
from rich import print as rprint

from .base import BaseObserver
from .util import is_observable


class BasicObserverDisc(BaseObserver):
    """離散時間状態観測器(p.72)"""

    def __init__(
        self,
        Ad: np.ndarray,
        Bd: np.ndarray,
        Cy: np.ndarray,
        Cz: np.ndarray,
        Ts: float,
        L: np.ndarray = None,
        pole: np.ndarray = None,
        init_x: np.ndarray = None,
    ) -> None:
        """
        Parameters
        ----------
        Ad, Bd, Cy, Cz: np.ndarray
            離散時間状態方程式の係数行列
        Ts: float
            サンプリング時間
        L: np.ndarray, default None
            オブザーバのゲイン行列
        pole: np.ndarray, default None
            Ad - L @ Cyの極
        init_x: np.ndarray, default None
            初期状態

        Returns
        ----------
        None

        Note
        ----------
        - Tsは使わない
        - L, poleはいずれか一方を指定
        """

        super().__init__()

        self._x_dim = Ad.shape[0]
        self._u_dim = Bd.shape[1]
        self._y_dim = Cy.shape[0]
        self._z_dim = Cz.shape[0]

        assert isinstance(Ad, np.ndarray)
        assert isinstance(Bd, np.ndarray)
        assert isinstance(Cy, np.ndarray)
        assert isinstance(Cz, np.ndarray)
        assert Ad.shape == (self._x_dim, self._x_dim)
        assert Bd.shape == (self._x_dim, self._u_dim)
        assert Cy.shape == (self._y_dim, self._x_dim)
        assert Cz.shape == (self._z_dim, self._x_dim)
        assert Ts > 0.0

        if init_x is None:
            self._x = np.zeros((self._x_dim,))
        else:
            assert init_x.shape == (self._x_dim,)
            self._x = init_x.copy()

        self._pole = None
        self._L = None
        if L is not None and pole is not None:
            raise TypeError('Please specify only one of "L" or "pole".')
        elif L is None and pole is not None:
            assert pole.shape == (self._x_dim,)
            assert np.all(abs(pole) < 1.0)
            assert is_observable(Ad, Cy), f"Rank(Mo) = {LA.matrix_rank(matlab.obsv(Ad, Cy))}"
            self._pole = pole.copy()
            self._L = np.array(matlab.place(Ad.T, Cy.T, pole).T)
        elif L is not None and pole is None:
            assert isinstance(L, np.ndarray)
            assert L.shape == (self._x_dim, self._y_dim)
            self._L = L
        else:
            self._L = np.zeros((self._x_dim, self._y_dim))

        self._A = Ad.copy()
        self._B = Bd.copy()
        self._Cy = Cy.copy()
        self._Cz = Cz.copy()
        self._K = Ad - self._L @ Cy
        self._Ts = Ts
        self._y = Cy @ self._x
        self._z = Cz @ self._x

        # 収束性の確認: Kの固有値が単位円の外にある場合は収束性が保証されない
        disc_eig, _ = LA.eig(self._K)
        max_abs_disc_eig = max(abs(disc_eig))
        if max_abs_disc_eig >= 1.0:
            rprint("[yellow]Warning: The system is unstable.[/yellow]")
            rprint(f"[yellow]eigenvalues of K: {disc_eig}[/yellow]")

    def step(self, y: np.ndarray, u: np.ndarray) -> None:
        assert y.shape == (self._y_dim,)
        assert u.shape == (self._u_dim,)

        self._x = self._K @ self._x + self._B @ u + self._L @ y
        self._y = self._Cy @ self._x
        self._z = self._Cz @ self._x

    def update_dynamics(self, Ad: np.ndarray, Bd: np.ndarray) -> None:
        assert isinstance(Ad, np.ndarray) and Ad.shape == (self._x_dim, self._x_dim)
        assert isinstance(Bd, np.ndarray) and Bd.shape == (self._x_dim, self._u_dim)

        if self._pole is None:
            rprint(
                "[yellow]Warning: The poles of L may change [/yellow]" "[yellow]because they depend on (A, C).[/yellow]"
            )
        else:
            self._L[:, :] = np.array(matlab.place(Ad.T, self._Cy.T, self._pole).T)

        self._A[:, :] = Ad
        self._B[:, :] = Bd
        self._K[:, :] = Ad - self._L @ self.Cy


class BasicObserverCont(BasicObserverDisc):
    """連続時間状態観測器(p.72)"""

    def __init__(
        self,
        Ac: np.ndarray,
        Bc: np.ndarray,
        Cy: np.ndarray,
        Cz: np.ndarray,
        Ts: float,
        L: np.ndarray = None,
        pole: np.ndarray = None,
        init_x: np.ndarray = None,
        disc_method: str = "euler",
    ) -> None:
        """
        Parameters
        ----------
        Ac, Bc, Cy, Cz: np.ndarray
            連続時間状態方程式の係数行列
        Ts: float
            サンプリング時間
        L: np.ndarray, default None
            離散時間オブザーバのゲイン行列
        pole: np.ndarray, default None
            離散時間オブザーバのAd - L @ Cyの極
        init_x: np.ndarray, default None
            初期状態
        disc_method: str, default 'euler'
            離散化手法

        Returns
        ----------
        None

        Note
        ----------
        - L, poleは離散時間のSSに対して適用されることに注意
        """

        self._disc_method = disc_method

        cont_sys = matlab.ss(Ac, Bc, Cy, np.zeros((Cy.shape[0], Bc.shape[1])))
        disc_sys = matlab.c2d(cont_sys, Ts, method=disc_method)

        super().__init__(
            Ad=np.array(disc_sys.A),  # .matrix -> np.ndarray
            Bd=np.array(disc_sys.B),  # np.asarray()だとこの型変換はされないことに注意
            Cy=Cy,
            Cz=Cz,
            Ts=Ts,
            L=L,
            pole=pole,
            init_x=init_x,
        )

    def update_dynamics(self, Ac: np.ndarray, Bc: np.ndarray) -> None:
        assert Ac.shape == (self.x_dim, self.x_dim)
        assert Bc.shape == (self.x_dim, self.u_dim)

        cont_sys = matlab.ss(Ac, Bc, self.Cy, np.zeros((self.Cy.shape[0], Bc.shape[1])))
        disc_sys = matlab.c2d(cont_sys, self._Ts, method=self._disc_method)

        super().update_dynamics(Ad=np.array(disc_sys.A), Bd=np.array(disc_sys.B))
