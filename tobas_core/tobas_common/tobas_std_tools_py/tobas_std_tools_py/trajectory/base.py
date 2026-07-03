# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

import numpy as np
from typing import Tuple
from abc import ABC, abstractmethod


class Trajectory(ABC):
    """Base class for multidimensional trajectory generators."""

    @abstractmethod
    def __call__(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate position, velocity, and acceleration.

        Parameters
        ----------
        t : float
            Time.

        Returns
        -------
        pos : np.ndarray[ndim=1]
            Position.
        vel : np.ndarray[ndim=1]
            Velocity.
        acc : np.ndarray[ndim=1]
            Acceleration.
        """
        raise NotImplementedError()

    @abstractmethod
    def get_duration(self) -> float:
        """Return the trajectory duration."""
        raise NotImplementedError()
