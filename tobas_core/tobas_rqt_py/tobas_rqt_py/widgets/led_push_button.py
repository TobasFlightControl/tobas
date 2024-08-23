import numpy as np
from typing import override, Tuple

from PyQt5.QtCore import QSize
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtGui import QMouseEvent, QResizeEvent

from .led_color import LEDColor


class LEDPushButton(QPushButton):
    CAPSULE = 1
    CIRCLE = 2
    RECTANGLE = 3

    def __init__(
        self,
        on_color: LEDColor = LEDColor.GREEN,
        off_color: LEDColor = LEDColor.BLACK,
        shape: int = RECTANGLE,
    ) -> None:
        super().__init__()

        self._qss = "QPushButton {{ \
                                   border: 2px solid lightgray; \
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
        self.__on_color: LEDColor = None
        self.__off_color: LEDColor = None
        self.__shape = None
        self.__height = 0

        self._on_color: LEDColor = on_color
        self._off_color: LEDColor = off_color
        self._shape = shape
        self._height = self.sizeHint().height()

        self.set_status(False)

    # ==================== Reimplemented Methods ====================
    @override
    def mousePressEvent(self, event: QMouseEvent) -> None:
        super().mousePressEvent(event)
        if self._status is False:
            self.set_status(True)
        else:
            self.set_status(False)

    @override
    def sizeHint(self) -> QSize:
        if self._shape == LEDPushButton.CAPSULE:
            return QSize(50, 30)
        elif self._shape == LEDPushButton.CIRCLE:
            return QSize(30, 30)
        elif self._shape == LEDPushButton.RECTANGLE:
            return QSize(40, 30)

    @override
    def resizeEvent(self, event: QResizeEvent) -> None:
        self._height = self.size().height()
        super().resizeEvent(event)

    @override
    def setFixedSize(self, width: int, height: int) -> None:
        self._height = height
        if self._shape == LEDPushButton.CIRCLE:
            super().setFixedSize(height, height)
        else:
            super().setFixedSize(width, height)

    # ==================== Properties ====================
    @property
    def _on_color(self) -> LEDColor:
        return self.__on_color

    @_on_color.setter
    def _on_color(self, color: LEDColor) -> None:
        self.__on_color = color
        self._update_on_qss()

    @_on_color.deleter
    def _on_color(self) -> None:
        del self.__on_color

    @property
    def _off_color(self) -> LEDColor:
        return self.__off_color

    @_off_color.setter
    def _off_color(self, color: LEDColor) -> None:
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

    # ==================== Methods ====================
    def set_on_color(self, color: LEDColor) -> None:
        self._on_color = color

    def set_off_color(self, color: LEDColor) -> None:
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
        color_str, grad_str = self._get_gradient(self.__on_color)
        self._on_qss = self._qss.format(self._end_radius, grad_str, color_str, color_str)

    def _update_off_qss(self) -> None:
        color_str, grad_str = self._get_gradient(self.__off_color)
        self._off_qss = self._qss.format(self._end_radius, grad_str, color_str, color_str)

    def _get_gradient(self, color: LEDColor) -> Tuple[str, str]:
        grad = ((LEDColor.WHITE.array - color.array) / 2).astype(np.uint8) + color.array
        grad_str = "{:02X}{:02X}{:02X}".format(grad[0], grad[1], grad[2])
        color_str = "{:02X}{:02X}{:02X}".format(color.array[0], color.array[1], color.array[2])
        return color_str, grad_str

    def _update_end_radius(self) -> None:
        if self.__shape == LEDPushButton.RECTANGLE:
            self._end_radius = int(self.__height / 10)
        else:
            self._end_radius = int(self.__height / 2)

    def _toggle_on(self) -> None:
        self.setStyleSheet(self._on_qss)

    def _toggle_off(self) -> None:
        self.setStyleSheet(self._off_qss)
