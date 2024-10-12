import numpy as np


class SimpleLPF:  # TODO: 未テスト
    def __init__(self, cutoff_freq: float, init_x: np.ndarray) -> None:
        assert cutoff_freq > 0.0
        assert init_x.ndim == 1

        self._cutoff_freq = cutoff_freq
        self._x = init_x.copy()
        self._dim = init_x.shape[0]

    def step(self, x: np.ndarray, dt: float) -> np.ndarray:
        assert x.shape == (self._dim,)
        assert dt > 0.0

        c = 1.0 / (1.0 + 2.0 * np.pi * dt * self._cutoff_freq)
        self._x = c * self._x + (1.0 - c) * x
        return self._x.copy()
