# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

from enum import Enum
from typing import List


class ExtEnum(Enum):
    """Enum class with extra methods."""

    @classmethod
    def names(cls) -> List[str]:
        """Return the list of all the names."""
        return [item.name for item in cls]

    @classmethod
    def values(cls) -> List[str]:
        """Return the list of all the values."""
        return [item.value for item in cls]
