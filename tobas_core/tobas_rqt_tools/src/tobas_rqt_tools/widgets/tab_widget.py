from overrides import override
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

    def __init__(self) -> None:
        super().__init__()
        self._index = 0

    def __iter__(self):
        return self

    def __next__(self) -> QWidget:
        if self._index < self.count():
            tab = self.widget(self._index)
            self._index += 1
            return tab
        else:
            raise StopIteration()

    def ignore_wheel_event(self) -> None:
        self.setTabBar(TabBarWithNoWheelEvent())

    def switch(self, tab: QWidget) -> None:
        idx = self.indexOf(tab)
        assert idx >= 0
        self.setCurrentIndex(idx)
