from typing import override
from PyQt5.QtCore import Qt, QRect, QPoint, QSize
from PyQt5.QtWidgets import QTabBar, QTabWidget, QStyle, QStylePainter, QStyleOptionTab
from PyQt5.QtGui import QWheelEvent

from .tab_widget import TabWidget


class VerticalTabBar(QTabBar):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

    @override
    def tabSizeHint(self, index: int) -> QSize:
        s = QTabBar.tabSizeHint(self, index)
        if s.width() < s.height():
            s.transpose()
        s.scale(s.width() * 2, s.height() * 2, Qt.AspectRatioMode.KeepAspectRatio)
        return s

    @override
    def paintEvent(self, _) -> None:
        painter = QStylePainter(self)
        style_option = QStyleOptionTab()

        for i in range(self.count()):
            self.initStyleOption(style_option, i)
            painter.drawControl(QStyle.ControlElement.CE_TabBarTabShape, style_option)
            painter.save()

            s = style_option.rect.size()
            s.scale(s.width() * 2, s.height() * 2, Qt.AspectRatioMode.KeepAspectRatio)
            rect = QRect(QPoint(), s)
            rect.moveCenter(style_option.rect.center())
            style_option.rect = rect

            center = self.tabRect(i).center()
            painter.translate(center)
            painter.rotate(90)
            painter.translate(center * -1)
            painter.drawControl(QStyle.ControlElement.CE_TabBarTabLabel, style_option)
            painter.restore()


class VerticalTabBarWithNoWheelEvent(VerticalTabBar):
    @override
    def wheelEvent(self, e: QWheelEvent) -> None:
        e.ignore()


class VerticalTabWidget(TabWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setTabBar(VerticalTabBar())
        self.setTabPosition(QTabWidget.TabPosition.West)

    @override
    def ignore_wheel_event(self) -> None:
        self.setTabBar(VerticalTabBarWithNoWheelEvent())
