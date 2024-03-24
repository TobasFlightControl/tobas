from typing import Sequence


def all_gt(seq: Sequence, x) -> bool:
    for elem in seq:
        if elem <= x:
            return False
    return True


def all_lt(seq: Sequence, x) -> bool:
    for elem in seq:
        if elem >= x:
            return False
    return True


def all_ge(seq: Sequence, x) -> bool:
    for elem in seq:
        if elem < x:
            return False
    return True


def all_le(seq: Sequence, x) -> bool:
    for elem in seq:
        if elem > x:
            return False
    return True


def is_unique(seq: Sequence):
    return len(seq) == len(set(seq))
