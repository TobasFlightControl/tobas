from typing import List


def is_unique(lst: List) -> bool:
    return len(lst) == len(set(lst))
