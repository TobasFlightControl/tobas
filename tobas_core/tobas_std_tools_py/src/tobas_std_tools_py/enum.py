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
