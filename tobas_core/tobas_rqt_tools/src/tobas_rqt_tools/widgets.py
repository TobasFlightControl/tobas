from __future__ import annotations
import os
import signal
import random
import markdown
import rospy
from typing import Callable
from overrides import override
from rviz import bindings as rviz
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_std_tools_py.config_parser import ConfigParserWrapper

from .utils import remap


class Widget(QWidget):
    """
    ===== QWidgetとの違い =====
    - closeで子ウィジェットのcloseを再帰的に呼び出す
    """

    @override
    def close(self) -> bool:
        rospy.logdebug(f"{self.__class__.__name__}.close")

        for child in self.findChildren(Widget):
            child.close()

        return super().close()


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

    @override
    def wheelEvent(self, e: QWheelEvent) -> None:
        e.ignore()

    @override
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

    @override
    def wheelEvent(self, e: QWheelEvent) -> None:
        e.ignore()

    @override
    def focusInEvent(self, e) -> None:
        super().focusInEvent(e)
        QTimer.singleShot(0, lambda: self.selectAll())


class ComboBox(QComboBox):
    """
    ===== QComboBoxとの違い =====
    - マウスホイールイベントを無効化
    - 追加メソッド
    """

    @override
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


class TableWidget(QTableWidget):
    """
    ===== QTableWidgetとの違い =====
    - 追加メソッド
    """

    def remove_all(self) -> None:
        """全ての行を削除する．clearとは異なり，内容に加えセルまで削除する．"""
        while self.rowCount() > 0:
            self.removeRow(0)


class Slider(QSlider):
    """
    ===== QSliderとの違い =====
    - マウスホイールイベントを無効化
    """

    @override
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

    @override
    def minimum(self) -> float:
        return self._min

    @override
    def setMinimum(self, minimum: float) -> None:
        self._min = minimum

    @override
    def maximum(self) -> float:
        return self._max

    @override
    def setMaximum(self, maximum: float) -> None:
        self._max = maximum

    @override
    def value(self) -> float:
        slider_value = super().value()
        return self._value_from_slider(slider_value)

    @override
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

    @override
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

    @override
    def __lt__(self, other: ListWidgetItem) -> bool:
        return self.data(Qt.UserRole) < other.data(Qt.UserRole)


class ScrollArea(QScrollArea):
    """
    ===== QScrollAreaとの違い =====
    - デフォルトでスクロール可能
    - setLayoutをオーバーライド
    """

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)

        self.setWidgetResizable(True)

    @override
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


class MainWidget(QWidget):
    """
    メイン画面
    - Configを作成
    - 最新のウィンドウ位置とサイズを保存
    """

    POS_X_KEY = "main_window/pos_x"
    POS_Y_KEY = "main_window/pos_y"
    WIDTH_KEY = "main_window/width"
    HEIGHT_KEY = "main_window/height"

    def __init__(self, config_path: str, section: str, title: str, icon_path: str, widget: QWidget) -> None:
        super().__init__()

        self.setWindowTitle(title)
        self.setWindowIcon(QIcon(icon_path))

        # ウィジェットを配置
        rows = QVBoxLayout()
        self.setLayout(rows)
        self._widget = widget
        rows.addWidget(self._widget)

        # configを読み込み
        self._config = ConfigParserWrapper(config_path, section)

        # 最新のウィンドウの位置とサイズを反映
        pos_x = self._config.getint(self.POS_X_KEY, fallback=-1)
        pos_y = self._config.getint(self.POS_Y_KEY, fallback=-1)
        width = self._config.getint(self.WIDTH_KEY, fallback=-1)
        height = self._config.getint(self.HEIGHT_KEY, fallback=-1)
        if pos_x >= 0 and pos_y >= 0 and width > 0 and height > 0:
            self.setGeometry(pos_x, pos_y, width, height)

    @override
    def closeEvent(self, _: QCloseEvent) -> None:
        rospy.logdebug(f"{self.__class__.__name__}.closeEvent")

        self._widget.close()

        # メインウィンドウが閉じられる時にプロセスごと落とすことで，確実に終了させる．
        os.kill(os.getpid(), signal.SIGINT)

    @override
    def moveEvent(self, event: QMoveEvent) -> None:
        # 現在のウィンドウ位置を保存
        self._config.read()
        cur_pos = self.pos()
        self._config.set(self.POS_X_KEY, cur_pos.x())
        self._config.set(self.POS_Y_KEY, cur_pos.y())
        self._config.write()

        return super().moveEvent(event)

    @override
    def resizeEvent(self, event: QResizeEvent) -> None:
        # 現在のウィンドウサイズを保存
        self._config.read()
        cur_size = self.size()
        self._config.set(self.WIDTH_KEY, cur_size.width())
        self._config.set(self.HEIGHT_KEY, cur_size.height())
        self._config.write()

        return super().resizeEvent(event)


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
