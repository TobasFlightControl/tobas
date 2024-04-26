from __future__ import annotations  # 自クラスをメソッドの引数としてアノテーションするために必要
from overrides import override
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QListWidget, QListWidgetItem


class ListWidget(QListWidget):
    """
    ===== QListWidgetItemとの違い =====
    - 追加メソッド
    """

    def remove(self, item: QListWidgetItem) -> None:
        row = self.row(item)
        self.takeItem(row)


class ListWidgetItem(QListWidgetItem):
    """
    ===== QListWidgetItemとの違い =====
    - UserRoleを基準にソート
    """

    @override
    def __lt__(self, other: ListWidgetItem) -> bool:
        return self.data(Qt.UserRole) < other.data(Qt.UserRole)
