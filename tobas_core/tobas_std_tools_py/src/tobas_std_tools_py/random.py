import numpy as np
import numpy.random as rnd
from typing import Tuple


def uniform(mean: float, std: float) -> float:
    assert std >= 0.0

    lb = mean - np.sqrt(3) * std
    ub = mean + np.sqrt(3) * std
    return rnd.uniform(lb, ub)


def sc_normal(mean: float, std: float, range_: Tuple[float, float] = None) -> float:
    if range_:
        assert len(range_) == 2
        lb, ub = min(range_), max(range_)
    else:
        lb, ub = -np.inf, np.inf

    assert lb <= mean <= ub
    assert std >= 0.0

    cnd = np.inf
    while cnd < lb or ub < cnd:
        cnd = rnd.normal(mean, std)
    return cnd


def sc_lognormal(mean: float, std: float, range_: Tuple[float, float] = None) -> float:
    if range_:
        assert len(range_) == 2
        lb, ub = min(range_), max(range_)
    else:
        lb, ub = -np.inf, np.inf

    assert lb <= mean <= ub
    assert std >= 0.0

    sign = np.sign(mean)
    mu = -np.log((std ** 2 / mean ** 2 + 1.0) / mean ** 2) / 2.0
    sigma = np.sqrt(2.0 * (np.log(sign * mean) - mu))

    cnd = np.inf
    while cnd < lb or ub < cnd:
        cnd = sign * rnd.lognormal(mu, sigma)
    return cnd


def sc_gamma(mean: float, std: float, range_: Tuple[float, float] = None) -> float:
    if range_:
        assert len(range_) == 2
        lb, ub = min(range_), max(range_)
    else:
        lb, ub = -np.inf, np.inf

    assert lb <= mean <= ub
    assert std >= 0.0

    sign = np.sign(mean)
    shape = mean ** 2 / std ** 2
    scale = sign * std ** 2 / mean

    cnd = np.inf
    while cnd < lb or ub < cnd:
        cnd = sign * rnd.gamma(shape, scale)
    return cnd
