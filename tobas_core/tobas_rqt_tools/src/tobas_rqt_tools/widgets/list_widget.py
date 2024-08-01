from __future__ import (
    annotations,
)  # 自クラスをメソッドの引数としてアノテーションするために必要
from overrides import override
from typing import Union, Iterator
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QDropEvent
from PyQt5.QtWidgets import QListWidget, QListWidgetItem


class ListWidget(QListWidget):
    """
    ===== QListWidgetItemとの違い =====
    - イテレータを定義
    - ドラッグアンドドロップでシグナル発行
    - 追加メソッド
    """

    item_moved = pyqtSignal(QListWidgetItem)

    def __iter__(self) -> Iterator[QListWidgetItem]:
        for row in range(self.count()):
            yield self.item(row)

    @override
    def dropEvent(self, event: QDropEvent) -> None:
        super().dropEvent(event)
        self.item_moved.emit(self.selectedItems()[0])

    def remove(self, item: QListWidgetItem) -> None:
        row = self.row(item)
        self.takeItem(row)

    def selected_item(self) -> Union[QListWidgetItem, None]:
        """選択中のアイテムのうち，最も上のものを返す．存在しない場合はNoneを返す．"""
        selected_items = self.selectedItems()
        return selected_items[0] if len(selected_items) > 0 else None


class ListWidgetItem(QListWidgetItem):
    """
    ===== QListWidgetItemとの違い =====
    - UserRoleを基準にソート
    """

    @override
    def __lt__(self, other: ListWidgetItem) -> bool:
        return self.data(Qt.UserRole) < other.data(Qt.UserRole)
