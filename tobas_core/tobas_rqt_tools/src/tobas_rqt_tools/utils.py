from PyQt5.QtCore import QObject


def remap(x: float, a: float, b: float, c: float, d: float) -> float:
    """xを[a, b]の範囲から[c, d]の範囲に投影する．"""
    assert a <= b
    assert c <= d

    if a == b:
        return (c + d) / 2
    else:
        return (c * (b - x) + d * (x - a)) / (b - a)


def block_signals_rec(obj: QObject, block: bool) -> None:
    """子ウィジェットを再帰的に走査し，全てのシグナルをブロックする．"""
    obj.blockSignals(block)
    for child in obj.children():
        block_signals_rec(child, block)
