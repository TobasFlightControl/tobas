from overrides import override
from typing import Iterator
from PyQt5.QtWidgets import QWidget, QTabBar, QTabWidget
from PyQt5.QtGui import QWheelEvent


class TabBarWithNoWheelEvent(QTabBar):
    @override
    def wheelEvent(self, e: QWheelEvent) -> None:
        e.ignore()


class TabWidget(QTabWidget):
    """
    ===== QtabWidgetとの違い =====
    - イテレータを定義
    - 追加メソッド
    """

    def __iter__(self) -> Iterator[QWidget]:
        for i in range(self.count()):
            yield self.widget(i)

    def ignore_wheel_event(self) -> None:
        self.setTabBar(TabBarWithNoWheelEvent())

    def switch(self, tab: QWidget) -> None:
        idx = self.indexOf(tab)
        assert idx >= 0
        self.setCurrentIndex(idx)
