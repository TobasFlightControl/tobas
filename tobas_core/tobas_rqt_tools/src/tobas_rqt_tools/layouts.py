from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


class FormLayout(QFormLayout):
    """
    ===== QFormLayoutとの違い =====
    - 追加メソッド
    """

    def clear(self) -> None:
        """全てのフォームを削除する．"""
        while self.rowCount() > 0:
            self.removeRow(0)

    def get_widget(self, row: int) -> QWidget:
        """指定した行に埋め込まれたウィジェットを取得する．"""
        item = self.itemAt(row, FormLayout.FieldRole)  # 2列目 (Field) を指定
        return item.widget()


def create_fixed_width_vboxlayout(width: int, parent: QBoxLayout) -> QVBoxLayout:
    widget = QWidget()
    widget.setFixedWidth(width)
    parent.addWidget(widget)

    res = QVBoxLayout()
    widget.setLayout(res)

    return res


def create_fixed_height_hboxlayout(height: int, parent: QBoxLayout) -> QHBoxLayout:
    widget = QWidget()
    widget.setFixedHeight(height)
    parent.addWidget(widget)

    res = QHBoxLayout()
    widget.setLayout(res)

    return res
