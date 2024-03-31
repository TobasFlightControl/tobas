from overrides import override
from PyQt5.QtWidgets import QWidget, QTabBar, QTabWidget
from PyQt5.QtGui import QWheelEvent


class _TabBar(QTabBar):
    """
    ===== QTabBarとの違い =====
    - マウスホイールイベントを無効化
    """

    @override
    def wheelEvent(self, e: QWheelEvent) -> None:
        e.ignore()


class TabWidget(QTabWidget):
    """
    ===== QtabWidgetとの違い =====
    - QTabBarのマウスホイールイベントを無効化
    - 追加メソッド
    """

    def __init__(self) -> None:
        super().__init__()
        self.setTabBar(_TabBar())

    def switch(self, tab: QWidget) -> None:
        idx = self.indexOf(tab)
        assert idx >= 0
        self.setCurrentIndex(idx)
