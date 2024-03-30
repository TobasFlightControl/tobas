import numpy as np
from overrides import override
from typing import Tuple

from PyQt5.QtCore import QSize
from PyQt5.QtWidgets import QWidget, QPushButton
from PyQt5.QtGui import QMouseEvent, QResizeEvent


class LEDWidget(QPushButton):
    BLACK = np.array([0x00, 0x00, 0x00], dtype=np.uint8)
    WHITE = np.array([0xFF, 0xFF, 0xFF], dtype=np.uint8)
    BLUE = np.array([0x73, 0xCE, 0xF4], dtype=np.uint8)
    GREEN = np.array([0xAD, 0xFF, 0x2F], dtype=np.uint8)
    ORANGE = np.array([0xFF, 0xA5, 0x00], dtype=np.uint8)
    PURPLE = np.array([0xAF, 0x00, 0xFF], dtype=np.uint8)
    RED = np.array([0xF4, 0x37, 0x53], dtype=np.uint8)
    YELLOW = np.array([0xFF, 0xFF, 0x00], dtype=np.uint8)

    CAPSULE = 1
    CIRCLE = 2
    RECTANGLE = 3

    def __init__(
        self,
        parent: QWidget = None,
        on_color: np.ndarray = GREEN,
        off_color: np.ndarray = BLACK,
        shape: int = RECTANGLE,
    ) -> None:
        super().__init__(parent=parent)

        self._qss = "QPushButton {{ \
                                   border: 3px solid lightgray; \
                                   border-radius: {}px; \
                                   background-color: \
                                       QLinearGradient( \
                                           y1: 0, y2: 1, \
                                           stop: 0 WHITE, \
                                           stop: 0.2 #{}, \
                                           stop: 0.8 #{}, \
                                           stop: 1 #{} \
                                       ); \
                                 }}"
        self._on_qss = ""
        self._off_qss = ""

        self._status = False
        self._end_radius = 0

        # Properties that will trigger changes on qss.
        self.__on_color = None
        self.__off_color = None
        self.__shape = None
        self.__height = 0

        self._on_color = on_color
        self._off_color = off_color
        self._shape = shape
        self._height = self.sizeHint().height()

        self.set_status(False)

    # =================================================== Reimplemented Methods
    @override
    def mousePressEvent(self, event: QMouseEvent) -> None:
        super().mousePressEvent(event)
        if self._status is False:
            self.set_status(True)
        else:
            self.set_status(False)

    @override
    def sizeHint(self) -> QSize:
        if self._shape == LEDWidget.CAPSULE:
            return QSize(50, 30)
        elif self._shape == LEDWidget.CIRCLE:
            return QSize(30, 30)
        elif self._shape == LEDWidget.RECTANGLE:
            return QSize(40, 30)

    @override
    def resizeEvent(self, event: QResizeEvent) -> None:
        self._height = self.size().height()
        super().resizeEvent(event)

    @override
    def setFixedSize(self, width: int, height: int) -> None:
        self._height = height
        if self._shape == LEDWidget.CIRCLE:
            super().setFixedSize(height, height)
        else:
            super().setFixedSize(width, height)

    @property
    def _on_color(self) -> np.ndarray:
        return self.__on_color

    @_on_color.setter
    def _on_color(self, color: np.ndarray) -> None:
        self.__on_color = color
        self._update_on_qss()

    @_on_color.deleter
    def _on_color(self) -> None:
        del self.__on_color

    @property
    def _off_color(self) -> np.ndarray:
        return self.__off_color

    @_off_color.setter
    def _off_color(self, color: np.ndarray) -> None:
        self.__off_color = color
        self._update_off_qss()

    @_off_color.deleter
    def _off_color(self) -> None:
        del self.__off_color

    @property
    def _shape(self) -> int:
        return self.__shape

    @_shape.setter
    def _shape(self, shape) -> None:
        self.__shape = shape
        self._update_end_radius()
        self._update_on_qss()
        self._update_off_qss()
        self.set_status(self._status)

    @_shape.deleter
    def _shape(self) -> None:
        del self.__shape

    @property
    def _height(self) -> int:
        return self.__height

    @_height.setter
    def _height(self, height) -> None:
        self.__height = height
        self._update_end_radius()
        self._update_on_qss()
        self._update_off_qss()
        self.set_status(self._status)

    @_height.deleter
    def _height(self) -> None:
        del self.__height

    # ================================================================= Methods
    def set_on_color(self, color: np.ndarray) -> None:
        self._on_color = color

    def set_off_color(self, color: np.ndarray) -> None:
        self._off_color = color

    def set_shape(self, shape: int) -> None:
        self._shape = shape

    def set_status(self, status: bool) -> None:
        self._status = True if status else False
        if self._status is True:
            self._toggle_on()
        else:
            self._toggle_off()

    def turn_on(self, status: bool = True) -> None:
        self.set_status(status)

    def turn_off(self, status: bool = False) -> None:
        self.set_status(status)

    def is_on(self) -> bool:
        return True if self._status is True else False

    def is_off(self) -> bool:
        return True if self._status is False else False

    def _update_on_qss(self) -> None:
        color, grad = self._get_gradient(self.__on_color)
        self._on_qss = self._qss.format(self._end_radius, grad, color, color)

    def _update_off_qss(self) -> None:
        color, grad = self._get_gradient(self.__off_color)
        self._off_qss = self._qss.format(self._end_radius, grad, color, color)

    def _get_gradient(self, color: np.ndarray) -> Tuple[str, str]:
        grad = ((self.WHITE - color) / 2).astype(np.uint8) + color
        grad = "{:02X}{:02X}{:02X}".format(grad[0], grad[1], grad[2])
        color = "{:02X}{:02X}{:02X}".format(color[0], color[1], color[2])
        return color, grad

    def _update_end_radius(self) -> None:
        if self.__shape == LEDWidget.RECTANGLE:
            self._end_radius = int(self.__height / 10)
        else:
            self._end_radius = int(self.__height / 2)

    def _toggle_on(self) -> None:
        self.setStyleSheet(self._on_qss)

    def _toggle_off(self) -> None:
        self.setStyleSheet(self._off_qss)
