from overrides import override
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QWidget, QSpinBox, QDoubleSpinBox
from PyQt5.QtGui import QWheelEvent


class SpinBox(QSpinBox):
    """
    ===== QSpinBoxとの違い =====
    - 最大最小のデフォルト値をint32の最大最小に設定
    - マウスホイールイベントを無効化
    - フォーカス時にテキスト全体を選択
    """

    INT32_MAX = (1 << 31) - 1
    INT32_MIN = -(1 << 31)

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)

        self.setMaximum(self.INT32_MAX)
        self.setMinimum(self.INT32_MIN)

    @override
    def wheelEvent(self, e: QWheelEvent) -> None:
        e.ignore()

    @override
    def focusInEvent(self, e) -> None:
        super().focusInEvent(e)
        QTimer.singleShot(0, lambda: self.selectAll())


class DoubleSpinBox(QDoubleSpinBox):
    """
    ===== QDoubleSpinBoxとの違い =====
    - 最大最小のデフォルト値を無限大に設定
    - マウスホイールイベントを無効化
    - フォーカス時にテキスト全体を選択
    """

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)

        self.setMaximum(float("inf"))
        self.setMinimum(float("-inf"))

    @override
    def wheelEvent(self, e: QWheelEvent) -> None:
        e.ignore()

    @override
    def focusInEvent(self, e) -> None:
        super().focusInEvent(e)
        QTimer.singleShot(0, lambda: self.selectAll())
