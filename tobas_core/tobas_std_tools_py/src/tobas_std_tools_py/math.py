import math


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
