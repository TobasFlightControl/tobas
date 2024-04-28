from __future__ import annotations  # 自クラスをメソッドの引数としてアノテーションするために必要
from overrides import override
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QDropEvent
from PyQt5.QtWidgets import QListWidget, QListWidgetItem


class ListWidget(QListWidget):
    """
    ===== QListWidgetItemとの違い =====
    - イテレータとして使用可能
    - ドラッグアンドドロップでシグナル発行
    - 追加メソッド
    """

    item_moved = pyqtSignal(QListWidgetItem)

    def __init__(self) -> None:
        super().__init__()
        self._index = 0

    def __iter__(self) -> ListWidget:
        self._index = 0
        return self

    def __next__(self) -> QListWidgetItem:
        if self._index >= self.count():
            raise StopIteration()

        item = self.item(self._index)
        self._index += 1
        return item

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
