from typing import Sequence, TypeVar

T = TypeVar("T", int, float)


def all_gt(seq: Sequence[T], x: T) -> bool:
    for elem in seq:
        if elem <= x:
            return False
    return True


def all_lt(seq: Sequence[T], x: T) -> bool:
    for elem in seq:
        if elem >= x:
            return False
    return True


def all_ge(seq: Sequence[T], x: T) -> bool:
    for elem in seq:
        if elem < x:
            return False
    return True


def all_le(seq: Sequence[T], x: T) -> bool:
    for elem in seq:
        if elem > x:
            return False
    return True


def is_unique(seq: Sequence):
    return len(seq) == len(set(seq))
