from PyQt5.QtCore import Qt, QObject
from PyQt5.QtWidgets import QWidget, QBoxLayout, QVBoxLayout, QHBoxLayout


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


def create_fixed_width_vboxlayout(width: int, parent: QBoxLayout) -> QVBoxLayout:
    widget = QWidget()
    widget.setFixedWidth(width)
    parent.addWidget(widget)

    res = QVBoxLayout()
    widget.setLayout(res)

    return res


def create_fixed_height_hboxlayout(height: int, parent: QBoxLayout) -> QHBoxLayout:
    widget = QWidget()
    widget.setFixedHeight(height)
    parent.addWidget(widget)

    res = QHBoxLayout()
    widget.setLayout(res)

    return res
