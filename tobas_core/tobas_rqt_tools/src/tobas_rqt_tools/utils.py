from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


def remap(x: float, a: float, b: float, c: float, d: float) -> float:
    """xを[a, b]の範囲から[c, d]の範囲に投影する．"""
    if a == b:
        return (c + d) / 2
    else:
        return (c * (b - x) + d * (x - a)) / (b - a)


def block_signals_rec(obj: QObject, block: bool) -> None:
    """子ウィジェットを再帰的に走査し，全てのシグナルをブロックする．"""
    obj.blockSignals(block)
    for child in obj.children():
        block_signals_rec(child, block)


def place_center(widget: QWidget, rows: QVBoxLayout) -> None:
    """ウィジェットをレイアウトの中央に配置する．"""
    cols = QHBoxLayout()
    cols.addWidget(widget)
    cols.setAlignment(widget, Qt.AlignCenter)
    rows.addLayout(cols)
