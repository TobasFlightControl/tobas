import numpy as np
from typing import Tuple
from PyQt5.QtWidgets import QWidget, QLabel

from .led_color import LEDColor


class LampWidget(QLabel):
    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent=parent)
        self._qss = "QLabel {{ \
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

    def set_color(self, color: LEDColor) -> None:
        end_radius = self.sizeHint().height() / 2
        color_str, grad_str = self._get_gradient(color)
        qss = self._qss.format(end_radius, grad_str, color_str, color_str)
        self.setStyleSheet(qss)

    def _get_gradient(self, color: LEDColor) -> Tuple[str, str]:
        grad = ((LEDColor.WHITE.array - color.array) / 2).astype(np.uint8) + color.array
        grad_str = "{:02X}{:02X}{:02X}".format(grad[0], grad[1], grad[2])
        color_str = "{:02X}{:02X}{:02X}".format(color.array[0], color.array[1], color.array[2])
        return color_str, grad_str
