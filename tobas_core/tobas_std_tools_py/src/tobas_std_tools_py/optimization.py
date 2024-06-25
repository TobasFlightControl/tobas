import numpy as np
import numpy.linalg as LA
from typing import Callable, List


def newton_1d(f: Callable, df: Callable, x0: float, eps: float = 1e-10, max_iter=1000):
    """1次元のニュートン法"""

    x = x0

    for _ in range(max_iter):
        div = df(x)
        assert div > 0.0
        x_new = x - f(x) / div
        if abs(x - x_new) < eps:
            break
        x = x_new
    else:
        print(f"Error: iteration limit exceeded.")

    return x


def newton(f: List[Callable], df: List[List[Callable]], x0: List[float], eps: float = 1e-10, max_iter: float = 1e-10):
    """多次元のニュートン法"""

    x = np.array(x0)

    for _ in range(max_iter):
        div = LA.inv(df(x))
        assert np.all(div > 0.0)
        x_new = x - np.dot(div, f(x))
        if np.sum((x - x_new) ** 2) < eps ** 2:
            break
        x = x_new
    else:
        print(f"Error: iteration limit exceeded.")

    return x
