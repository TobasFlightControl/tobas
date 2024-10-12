import numpy as np
import numpy.linalg as LA
from control import matlab


def is_controllable(A: np.ndarray, B: np.ndarray) -> bool:
    x_dim = A.shape[0]
    u_dim = B.shape[1]

    assert A.shape == (x_dim, x_dim)
    assert B.shape == (x_dim, u_dim)

    Mc = matlab.ctrb(A, B)
    rank = LA.matrix_rank(Mc)
    return rank == x_dim


def is_observable(A: np.ndarray, C: np.ndarray) -> bool:
    x_dim = A.shape[0]
    y_dim = C.shape[0]

    assert A.shape == (x_dim, x_dim)
    assert C.shape == (y_dim, x_dim)

    Mo = matlab.obsv(A, C)
    rank = LA.matrix_rank(Mo)
    return rank == x_dim
