from __future__ import annotations  # 自クラスをメソッドの引数としてアノテーションするために必要
from overrides import override
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QDropEvent
from PyQt5.QtWidgets import QListWidget, QListWidgetItem


class ListWidget(QListWidget):
    """
    ===== QListWidgetItemとの違い =====
    - ドラッグアンドドロップでシグナル発行
    - 追加メソッド
    """

    item_moved = pyqtSignal(QListWidgetItem)

    @override
    def dropEvent(self, event: QDropEvent) -> None:
        super().dropEvent(event)
        self.item_moved.emit(self.selectedItems()[0])

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
