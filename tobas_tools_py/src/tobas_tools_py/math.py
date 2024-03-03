import math


def rpm2rps(rpm: float) -> float:
    """RPM -> rad/s"""
    return (math.pi / 30) * rpm


def rps2rpm(rpm: float) -> float:
    """rad/s -> RPM"""
    return (30 / math.pi) * rpm
