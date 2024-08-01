import math
from typing import Tuple, List, Union


def rpm2rps(rpm: float) -> float:
    """RPM -> rad/s"""
    return (math.pi / 30) * rpm


def rps2rpm(rpm: float) -> float:
    """rad/s -> RPM"""
    return (30 / math.pi) * rpm


def remap(x: float, a: float, b: float, c: float, d: float) -> float:
    """xを[a, b]の範囲から[c, d]の範囲に投影する．"""
    assert math.isfinite(x)

    if a == b:
        return (c + d) / 2
    else:
        return (c * (b - x) + d * (x - a)) / (b - a)


def wrap(x, n):
    """与えられた数を[-n, n]の範囲にラップする．"""
    n2 = 2 * n
    if x >= 0:
        while x > n:
            x -= n2
    else:
        while x < -n:
            x += n2
    return x


def ceil(x: float, unit: int = 1) -> int:
    return math.ceil(x / unit) * unit


def floor(x: float, unit: int = 1) -> int:
    return math.floor(x / unit) * unit


def is_almost_int(x: float) -> bool:
    """小数xが整数に近いかどうかを判定する"""

    r = abs(x - round(x))
    return r < 1e-5


def common_range(range_list: List[Tuple[float, float]]) -> Union[Tuple[float, float], None]:
    """range_listの共通範囲を求める"""

    lb_res = -math.inf
    ub_res = math.inf

    for lb, ub in range_list:
        assert lb <= ub
        lb_res = max(lb_res, lb)
        ub_res = min(ub_res, ub)
        if lb_res > ub_res:
            return None

    return lb_res, ub_res
