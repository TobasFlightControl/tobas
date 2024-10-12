from typing import Iterator, Tuple
from PyQt5.QtWidgets import QWidget, QFormLayout


class FormLayout(QFormLayout):
    """
    ===== QFormLayoutとの違い =====
    - イテレータを定義
    - 追加メソッド
    """

    def __iter__(self) -> Iterator[Tuple[QWidget, QWidget]]:
        for row in range(self.rowCount()):
            yield self.get_label(row), self.get_widget(row)

    def clear(self) -> None:
        """全てのフォームを削除する．"""
        while self.rowCount() > 0:
            self.removeRow(0)

    def get_label(self, row: int) -> QWidget:
        """指定した行のラベルを取得する．"""
        item = self.itemAt(row, QFormLayout.ItemRole.LabelRole)
        return item.widget()

    def get_widget(self, row: int) -> QWidget:
        """指定した行のウィジェットを取得する．"""
        item = self.itemAt(row, QFormLayout.ItemRole.FieldRole)
        return item.widget()
