import sys
from abc import abstractmethod
from overrides import override
from PyQt5.QtCore import Qt, QRect, QTimer
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QPaintEvent, QPen, QFont

from tobas_std_tools_py.math import remap


class RangeWidget(QWidget):
    DEFAULT_LINE_WIDTH = 3
    DEFAULT_TEXT_PSIZE = 10
    DEFAULT_UPDATE_RATE = 10  # [Hz] リフレッシュレート

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)

        self._minimum = 0
        self._maximum = 0
        self._text = ""
        self._line_width = 3
        self._text_psize = 10

        self._value = None
        self._lower = sys.maxsize
        self._upper = -sys.maxsize

        # 再描画の周波数が高すぎるとフリーズするため，周波数を抑えて一定周期で描画する．
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.update)

    def set_minimum(self, minimum: float) -> None:
        self._minimum = minimum

    def set_maximum(self, maximum: float) -> None:
        self._maximum = maximum

    def set_text(self, text: str) -> None:
        self._text = text

    def get_value(self) -> float:
        return self._value

    def set_value(self, value: float) -> None:
        self._value = value
        if value < self._lower:
            self._lower = value
        if value > self._upper:
            self._upper = value

    def get_lower(self) -> float:
        return self._lower

    def set_lower(self, lower: float) -> None:
        self._lower = lower

    def get_upper(self) -> float:
        return self._upper

    def set_upper(self, upper: float) -> None:
        self._upper = upper

    def get_middle(self) -> float:
        return (self._lower + self._upper) / 2

    def clear(self) -> None:
        self._value = None
        self._lower = self._maximum
        self._upper = self._minimum
        self.update()

    def start_timer(self, update_rate: int = DEFAULT_UPDATE_RATE) -> None:
        assert update_rate > 0
        self._timer.start(1000 / update_rate)

    def stop_timer(self) -> None:
        self._timer.stop()

    @abstractmethod
    def paintEvent(self, event: QPaintEvent) -> None:
        raise NotImplementedError()


class HRangeWidget(RangeWidget):
    @override
    def paintEvent(self, event: QPaintEvent) -> None:
        # QPainterはpaintEvent内でのみ定義できる
        painter = QPainter(self)

        # 背景を描画
        painter.fillRect(event.rect(), Qt.white)

        # 枠を描画
        painter.setPen(Qt.black)
        painter.drawRect(0, 0, self.width(), self.height())

        if self._lower < self._upper:
            # バーの位置を計算
            lower_pos = remap(self._lower, self._minimum, self._maximum, 0, self.width())
            upper_pos = remap(self._upper, self._minimum, self._maximum, 0, self.width())

            # 最小値と最大値の間を緑色で塗る
            painter.setBrush(Qt.green)
            painter.drawRect(lower_pos, 0, upper_pos - lower_pos, self.height())

            # 最小値と最大値の位置に黒色の線を描画
            painter.setPen(QPen(Qt.black, self._line_width))
            painter.drawLine(lower_pos, 0, lower_pos, self.height())
            painter.drawLine(upper_pos, 0, upper_pos, self.height())

        if self._value is not None:
            # バーの位置を計算
            value_pos = remap(self._value, self._minimum, self._maximum, 0, self.width())

            # 現在値の位置に赤色の線を描画
            painter.setPen(QPen(Qt.red, self._line_width))
            painter.drawLine(value_pos, 0, value_pos, self.height())

        # テキストを描画
        if self._text:
            painter.setPen(Qt.gray)
            painter.setFont(QFont("Default", self._text_psize))
            painter.drawText(QRect(0, 0, self.width(), self.height()), Qt.AlignCenter, self._text)

        # Painterを破棄 (適切に破棄しないとメモリリークが起きる)
        painter.end()


class VRangeWidget(RangeWidget):
    @override
    def paintEvent(self, event: QPaintEvent) -> None:
        # QPainterはpaintEvent内でのみ定義できる
        painter = QPainter(self)

        # 背景を描画
        painter.fillRect(event.rect(), Qt.white)

        # 枠を描画
        painter.setPen(Qt.black)
        painter.drawRect(0, 0, self.width(), self.height())

        if self._lower < self._upper:
            # バーの位置を計算
            lower_pos = remap(self._lower, self._minimum, self._maximum, 0, self.height())
            upper_pos = remap(self._upper, self._minimum, self._maximum, 0, self.height())

            # 最小値と最大値の間を緑色で塗る
            painter.setBrush(Qt.green)
            painter.drawRect(0, lower_pos, self.width(), upper_pos - lower_pos)

            # 最小値と最大値の位置に黒色の線を描画
            painter.setPen(QPen(Qt.black, self._line_width))
            painter.drawLine(0, lower_pos, self.width(), lower_pos)
            painter.drawLine(0, upper_pos, self.width(), upper_pos)

        if self._value is not None:
            # バーの位置を計算
            value_pos = remap(self._value, self._minimum, self._maximum, 0, self.height())

            # 現在値の位置に赤色の線を描画
            painter.setPen(QPen(Qt.red, self._line_width))
            painter.drawLine(0, value_pos, self.width(), value_pos)

        # テキストを描画
        if self._text:
            # フォントを設定
            painter.setPen(Qt.gray)
            painter.setFont(QFont("Default", self._text_psize))

            # ペインターの回転と移動を設定
            painter.translate(self.width() / 2, self.height() / 2)
            painter.rotate(90)

            # 回転した状態でテキストを描画
            text_rect = QRect(-self.height() / 2, -self.width() / 2, self.height(), self.width())
            painter.drawText(text_rect, Qt.AlignCenter, self._text)

        # Painterを破棄 (適切に破棄しないとメモリリークが起きる)
        painter.end()
