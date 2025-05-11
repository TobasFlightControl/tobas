from typing import Callable

import numpy as np
from numpy import linalg as LA
from numpy.typing import NDArray


def newton_1d(f: Callable, df: Callable, x0: float, eps: float = 1e-10, max_iter: int = 100) -> float:
    """1次元のニュートン法"""

    x = x0

    for _ in range(max_iter):
        dx = -f(x) / df(x)
        x += dx
        if abs(dx) < eps:
            break
    else:
        print("Error: Iteration limit exceeded.")

    return x


def newton(
    f: Callable,
    df: Callable,
    x0: NDArray,
    eps: float = 1e-10,
    max_iter: int = 100,
) -> NDArray:
    """多次元のニュートン法"""

    x = np.array(x0)

    for _ in range(max_iter):
        dx = -np.dot(LA.inv(df(x)), f(x))
        if np.sum(dx**2) < eps**2:
            break
        x += dx
    else:
        print("Error: Iteration limit exceeded.")

    return x
