import numpy as np
from abc import ABC, abstractmethod


class BaseObserver(ABC):

    @abstractmethod
    def __init__(self) -> None:
        raise NotImplementedError()

    @abstractmethod
    def step(self, y: np.ndarray, u: np.ndarray) -> None:
        raise NotImplementedError()

    @abstractmethod
    def update_dynamics(
        self,
        A: np.ndarray,
        B: np.ndarray,
        Cy: np.ndarray,
        Cz: np.ndarray,
    ) -> None:
        raise NotImplementedError()

    @property
    def Ts(self) -> float:
        return self._Ts

    @property
    def x_dim(self) -> int:
        return self._x_dim

    @property
    def u_dim(self) -> int:
        return self._u_dim

    @property
    def y_dim(self) -> int:
        return self._y_dim

    @property
    def z_dim(self) -> int:
        return self._z_dim

    @property
    def A(self) -> np.ndarray:
        return self._A

    @property
    def B(self) -> np.ndarray:
        return self._B

    @property
    def Cy(self) -> np.ndarray:
        return self._Cy

    @property
    def Cz(self) -> np.ndarray:
        return self._Cz

    @property
    def L(self) -> np.ndarray:
        return self._L

    @property
    def x(self) -> np.ndarray:
        return self._x

    @property
    def y(self) -> np.ndarray:
        return self._y

    @property
    def z(self) -> np.ndarray:
        return self._z
