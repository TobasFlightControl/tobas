import sys
import math
from abc import abstractmethod
from overrides import override
from typing import Union
from PyQt5.QtCore import Qt, QRect, QTimer
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QPaintEvent, QPen, QFont

from tobas_std_tools_py.math import remap


class PositionBarWidget(QWidget):
    DEFAULT_LINE_WIDTH = 3
    DEFAULT_TEXT_PSIZE = 10
    DEFAULT_UPDATE_RATE = 10  # [Hz] リフレッシュレート

    def __init__(
        self,
        fill_range: bool = True,
        minimum: float = 0.0,
        maximum: float = 0.0,
        text: str = "",
        line_width: int = DEFAULT_LINE_WIDTH,
        text_psize: int = DEFAULT_TEXT_PSIZE,
    ) -> None:
        super().__init__()

        self._fill_range = fill_range
        self._minimum = minimum
        self._maximum = maximum
        self._text = text
        self._line_width = line_width
        self._text_psize = text_psize

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

    def set_line_width(self, line_width: int) -> None:
        self._line_width = line_width

    def set_text_psize(self, text_psize: int) -> None:
        self._text_psize = text_psize

    def get_value(self) -> Union[float, None]:
        return self._value

    def set_value(self, value: float) -> None:
        assert math.isfinite(value)
        self._value = value
        if value < self._lower:
            self._lower = value
        if value > self._upper:
            self._upper = value

    def get_lower(self) -> float:
        return self._lower

    def set_lower(self, lower: float) -> None:
        assert math.isfinite(lower)
        self._lower = lower

    def get_upper(self) -> float:
        return self._upper

    def set_upper(self, upper: float) -> None:
        assert math.isfinite(upper)
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

    @override
    def paintEvent(self, event: QPaintEvent) -> None:
        # QPainterはpaintEvent内でのみ定義できる
        painter = QPainter(self)

        # 背景を描画
        painter.fillRect(event.rect(), Qt.white)

        # 枠を描画
        painter.setPen(Qt.black)
        painter.drawRect(0, 0, self.width(), self.height())

        # 値の範囲を塗りつぶす
        if self._fill_range and self._lower < self._upper:
            self._draw_range(painter)

        # 値の位置を表示
        if self._value is not None:
            self._draw_value(painter)

        # テキストを表示
        if self._text:
            self._draw_text(painter)

        # Painterを破棄 (適切に破棄しないとメモリリークが起きる)
        painter.end()

    @abstractmethod
    def _draw_range(self, painter: QPainter) -> None:
        raise NotImplementedError()

    @abstractmethod
    def _draw_value(self, painter: QPainter) -> None:
        raise NotImplementedError()

    @abstractmethod
    def _draw_text(self, painter: QPainter) -> None:
        raise NotImplementedError()


class HPositionBarWidget(PositionBarWidget):
    @override
    def _draw_range(self, painter: QPainter) -> None:
        # バーの位置を計算
        lower_pos = int(remap(self._lower, self._minimum, self._maximum, 0, self.width()))
        upper_pos = int(remap(self._upper, self._minimum, self._maximum, 0, self.width()))

        # 最小値と最大値の間を緑色で塗る
        painter.setBrush(Qt.green)
        painter.drawRect(lower_pos, 0, upper_pos - lower_pos, self.height())

        # 最小値と最大値の位置に黒色の線を描画
        painter.setPen(QPen(Qt.black, self._line_width))
        painter.drawLine(lower_pos, 0, lower_pos, self.height())
        painter.drawLine(upper_pos, 0, upper_pos, self.height())

    @override
    def _draw_value(self, painter: QPainter) -> None:
        # バーの位置を計算
        value_pos = int(remap(self._value, self._minimum, self._maximum, 0, self.width()))

        # 現在値の位置に赤色の線を描画
        painter.setPen(QPen(Qt.red, self._line_width))
        painter.drawLine(value_pos, 0, value_pos, self.height())

    @override
    def _draw_text(self, painter: QPainter) -> None:
        painter.setPen(Qt.gray)
        painter.setFont(QFont("Default", self._text_psize))
        painter.drawText(QRect(0, 0, self.width(), self.height()), Qt.AlignCenter, self._text)


class VPositionBarWidget(PositionBarWidget):
    @override
    def _draw_range(self, painter: QPainter) -> None:
        # バーの位置を計算
        lower_pos = int(remap(self._lower, self._minimum, self._maximum, 0, self.height()))
        upper_pos = int(remap(self._upper, self._minimum, self._maximum, 0, self.height()))

        # 最小値と最大値の間を緑色で塗る
        painter.setBrush(Qt.green)
        painter.drawRect(0, lower_pos, self.width(), upper_pos - lower_pos)

        # 最小値と最大値の位置に黒色の線を描画
        painter.setPen(QPen(Qt.black, self._line_width))
        painter.drawLine(0, lower_pos, self.width(), lower_pos)
        painter.drawLine(0, upper_pos, self.width(), upper_pos)

    @override
    def _draw_value(self, painter: QPainter) -> None:
        # バーの位置を計算
        value_pos = int(remap(self._value, self._minimum, self._maximum, 0, self.height()))

        # 現在値の位置に赤色の線を描画
        painter.setPen(QPen(Qt.red, self._line_width))
        painter.drawLine(0, value_pos, self.width(), value_pos)

    @override
    def _draw_text(self, painter: QPainter) -> None:
        # フォントを設定
        painter.setPen(Qt.gray)
        painter.setFont(QFont("Default", self._text_psize))

        # ペインターの回転と移動を設定
        painter.translate(self.width() / 2, self.height() / 2)
        painter.rotate(90)

        # 回転した状態でテキストを描画
        text_rect = QRect(-self.height() / 2, -self.width() / 2, self.height(), self.width())
        painter.drawText(text_rect, Qt.AlignCenter, self._text)
