from __future__ import annotations
import os
import os.path as osp
import random
import markdown
import rospy
from typing import Callable
from overrides import overrides
from configparser import ConfigParser
from rviz import bindings as rviz
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from .utils import remap


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

    @overrides
    def wheelEvent(self, e: QWheelEvent) -> None:
        e.ignore()

    @overrides
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

    @overrides
    def wheelEvent(self, e: QWheelEvent) -> None:
        e.ignore()

    @overrides
    def focusInEvent(self, e) -> None:
        super().focusInEvent(e)
        QTimer.singleShot(0, lambda: self.selectAll())


class ComboBox(QComboBox):
    """
    ===== QComboBoxとの違い =====
    - マウスホイールイベントを無効化
    - 追加メソッド
    """

    @overrides
    def wheelEvent(self, e: QWheelEvent) -> None:
        e.ignore()

    def contains(self, text: str) -> bool:
        for idx in range(self.count()):
            if self.itemText(idx) == text:
                return True
        return False

    def remove_text(self, text: str) -> None:
        idx = self.findText(text)
        self.removeItem(idx)


class Slider(QSlider):
    """
    ===== QSliderとの違い =====
    - マウスホイールイベントを無効化
    """

    @overrides
    def wheelEvent(self, e: QWheelEvent) -> None:
        e.ignore()


class FloatSlider(QSlider):
    """小数を扱うスライダー．"""

    RANGE = 10000
    DEFAULT_MINIMUM = 0.0
    DEFAULT_MAXIMUM = 1.0

    valueChanged = pyqtSignal(float)  # シグナルのオーバーライドも可能

    def __init__(self, orientation: Qt.Orientation, parent: QWidget = None) -> None:
        super().__init__(orientation, parent)

        self._min = self.DEFAULT_MINIMUM
        self._max = self.DEFAULT_MAXIMUM

        super().setRange(0, self.RANGE)
        super().setValue(self.RANGE // 2)

        super().valueChanged.connect(self._on_slider_value_changed)

    @overrides
    def minimum(self) -> float:
        return self._min

    @overrides
    def setMinimum(self, minimum: float) -> None:
        self._min = minimum

    @overrides
    def maximum(self) -> float:
        return self._max

    @overrides
    def setMaximum(self, maximum: float) -> None:
        self._max = maximum

    @overrides
    def value(self) -> float:
        slider_value = super().value()
        return self._value_from_slider(slider_value)

    @overrides
    def setValue(self, value: float) -> None:
        slider_value = int(remap(value, self._min, self._max, 0.0, self.RANGE))
        super().setValue(slider_value)

    @pyqtSlot(int)
    def _on_slider_value_changed(self, slider_value: int) -> None:
        value = self._value_from_slider(slider_value)
        self.valueChanged.emit(value)

    def _value_from_slider(self, slider_value: int) -> float:
        return remap(float(slider_value), 0.0, self.RANGE, self._min, self._max)


class TabBar(QTabBar):
    """
    ===== QTabBarとの違い =====
    - マウスホイールイベントを無効化
    """

    @overrides
    def wheelEvent(self, e: QWheelEvent) -> None:
        e.ignore()


class TabWidget(QTabWidget):
    """
    ===== QtabWidgetとの違い =====
    - QTabBarのマウスホイールイベントを無効化
    """

    def __init__(self) -> None:
        super().__init__()
        self.setTabBar(TabBar())


class ListWidgetItem(QListWidgetItem):
    """
    ===== QListWidgetItemとの違い =====
    - UserRoleを基準にソート
    """

    @overrides
    def __lt__(self, other: ListWidgetItem) -> bool:
        return self.data(Qt.UserRole) < other.data(Qt.UserRole)


class ScrollArea(QScrollArea):
    """
    ===== QScrollAreaとの違い =====
    - setLayoutをオーバーライド
    """

    @overrides
    def setLayout(self, layout: QLayout) -> None:
        # デフォルトのsetLayoutは親クラスであるQWidgetの名残であり，そのままでは使用できない
        # スクロールエリアに入れられるウィジェットは1つのみだから，Layoutを使うためには空のウィジェットを挟む必要がある
        inner_widget = QWidget()
        self.setWidget(inner_widget)
        inner_widget.setLayout(layout)


class MarkDownWidget(QTextBrowser):
    """マークダウン形式のテキストを表示するウィジェット．数式は書けない．"""

    def __init__(self) -> None:
        super().__init__()

        # 背景色を透明にし，枠線を消す設定
        # これで親ウィジェットに自然にマークダウンテキストを組み込める
        self.setStyleSheet("background-color: transparent; border: none")

    def set_text(self, markdown_text: str) -> None:
        html = markdown.markdown(markdown_text)
        self.setHtml(html)


class MainWidget(ScrollArea):
    """
    メイン画面
    - Configを作成
    - 最新のウィンドウ位置とサイズを保存
    - 終了時にROSノードを落とす
    """

    POS_X_KEY = "main_window/pos_x"
    POS_Y_KEY = "main_window/pos_y"
    WIDTH_KEY = "main_window/width"
    HEIGHT_KEY = "main_window/height"

    def __init__(self, name: str, section: str = "DEFAULT") -> None:
        super().__init__()

        self._config_path = osp.expanduser(f"~/.config/{name}/config.ini")
        self._section = section

        # スクロール可能にする
        self.setWidgetResizable(True)

        # configがなければ作成
        config_dir = osp.dirname(self._config_path)
        os.makedirs(config_dir, exist_ok=True)

        # 最新のウィンドウの位置とサイズを反映
        self._config = ConfigParser()
        self._config.read(self._config_path)
        pos_x = self._config.getint(self._section, self.POS_X_KEY, fallback=-1)
        pos_y = self._config.getint(self._section, self.POS_Y_KEY, fallback=-1)
        width = self._config.getint(self._section, self.WIDTH_KEY, fallback=-1)
        height = self._config.getint(self._section, self.HEIGHT_KEY, fallback=-1)
        if pos_x >= 0 and pos_y >= 0 and width > 0 and height > 0:
            self.setGeometry(pos_x, pos_y, width, height)

    @overrides
    def moveEvent(self, event: QMoveEvent) -> None:
        # 現在のウィンドウ位置を保存
        self._config.read(self._config_path)
        cur_pos = self.pos()
        self._config[self._section][self.POS_X_KEY] = str(cur_pos.x())
        self._config[self._section][self.POS_Y_KEY] = str(cur_pos.y())
        with open(self._config_path, "w") as f:
            self._config.write(f)

        return super().moveEvent(event)

    @overrides
    def resizeEvent(self, event: QResizeEvent) -> None:
        # 現在のウィンドウサイズを保存
        self._config.read(self._config_path)
        cur_size = self.size()
        self._config[self._section][self.WIDTH_KEY] = str(cur_size.width())
        self._config[self._section][self.HEIGHT_KEY] = str(cur_size.height())
        with open(self._config_path, "w") as f:
            self._config.write(f)

        return super().resizeEvent(event)

    @overrides
    def closeEvent(self, event: QCloseEvent) -> None:
        rospy.signal_shutdown("Main window is closed.")
        return super().closeEvent(event)


class IntSliderDisplay(QWidget):
    """整数スライダーとその値の表示機能を持つウィジェット．"""

    PSIZE = 9

    value_changed = pyqtSignal(int)

    def __init__(self, parent: QObject = None) -> None:
        super().__init__(parent)
        self._suffix = ""

        font = QFont("Default", self.PSIZE, QFont.Bold)

        rows = QVBoxLayout()
        self.setLayout(rows)

        cols = QHBoxLayout()
        rows.addLayout(cols)

        self._text = QLabel(self)
        self._text.setFont(font)
        cols.addWidget(self._text)

        self._value = QLineEdit(self)
        self._value.setAlignment(Qt.AlignRight)
        self._value.setFont(font)
        self._value.setReadOnly(True)
        self._value.setFocusPolicy(Qt.NoFocus)
        cols.addWidget(self._value)

        self._slider = Slider(Qt.Horizontal, self)
        rows.addWidget(self._slider)

        self.update()
        self._slider.valueChanged.connect(self._on_value_changed)

    def update(self) -> None:
        self._value.setText(f"{self.get_value()}{self._suffix}")
        self.value_changed.emit(self.get_value())

    def set_text(self, text: str) -> None:
        self._text.setText(text)

    def set_minimum(self, minimum: int) -> None:
        self._slider.setMinimum(minimum)

    def set_maximum(self, maximum: int) -> None:
        self._slider.setMaximum(maximum)

    def get_value(self) -> int:
        return self._slider.value()

    def set_value(self, value: int) -> None:
        self._slider.setValue(value)

    def set_suffix(self, suffix: str) -> None:
        self._suffix = suffix
        self.update()

    def set_callback(self, callback: Callable[[int], None]):
        self.value_changed.connect(callback)

    def set_random_value(self) -> None:
        value = random.randint(self._slider.minimum(), self._slider.maximum())
        self.set_value(value)

    def set_center_value(self) -> None:
        value = (self._slider.minimum() + self._slider.maximum()) // 2
        self.set_value(value)

    @pyqtSlot()
    def _on_value_changed(self) -> None:
        self.update()


class FloatSliderDisplay(QWidget):
    """小数スライダーとその値の表示機能を持つウィジェット．"""

    PSIZE = 9

    value_changed = pyqtSignal(float)

    def __init__(self, parent: QObject = None) -> None:
        super().__init__(parent)
        self._suffix = ""

        font = QFont("Default", self.PSIZE, QFont.Bold)

        rows = QVBoxLayout()
        self.setLayout(rows)

        cols = QHBoxLayout()
        rows.addLayout(cols)

        self._text = QLabel(self)
        self._text.setFont(font)
        cols.addWidget(self._text)

        self._value = QLineEdit(self)
        self._value.setAlignment(Qt.AlignRight)
        self._value.setFont(font)
        self._value.setReadOnly(True)
        self._value.setFocusPolicy(Qt.NoFocus)
        cols.addWidget(self._value)

        self._slider = FloatSlider(Qt.Horizontal, self)
        rows.addWidget(self._slider)

        self.update()
        self._slider.valueChanged.connect(self._on_value_changed)

    def update(self) -> None:
        self._value.setText(f"{self.get_value()}{self._suffix}")
        self.value_changed.emit(self.get_value())

    def set_text(self, text: str) -> None:
        self._text.setText(text)

    def set_minimum(self, minimum: float) -> None:
        self._slider.setMinimum(minimum)

    def set_maximum(self, maximum: float) -> None:
        self._slider.setMaximum(maximum)

    def get_value(self) -> float:
        return self._slider.value()

    def set_value(self, value: float) -> None:
        self._slider.setValue(value)

    def set_suffix(self, suffix: str) -> None:
        self._suffix = suffix

    def set_callback(self, callback: Callable[[float], None]) -> None:
        self.value_changed.connect(callback)

    def set_random_value(self) -> None:
        value = random.uniform(self._slider.minimum(), self._slider.maximum())
        self.set_value(value)

    def set_center_value(self) -> None:
        value = (self._slider.minimum() + self._slider.maximum()) / 2
        self.set_value(value)

    @pyqtSlot()
    def _on_value_changed(self) -> None:
        self.update()


def create_rviz_frame(config_path: str):
    # Setup frame
    # cf. RViz Python Tutorial: https://docs.ros.org/en/indigo/api/rviz_python_tutorial/html/
    reader = rviz.YamlConfigReader()
    config = rviz.Config()
    reader.readFile(config, config_path)

    # Setup Visualization Frame
    # https://docs.ros.org/en/jade/api/rviz/html/c++/visualization__frame_8h_source.html
    frame = rviz.VisualizationFrame()
    frame.setSplashPath("")
    frame.initialize()
    frame.load(config)
    frame.setMenuBar(None)
    frame.setStatusBar(None)
    frame.setHideButtonVisibility(False)
    frame.setStyleSheet("QSizeGrip { width: 0px; height: 0px; }")  # Remove sizegrip

    return frame
