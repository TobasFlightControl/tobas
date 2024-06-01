from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .urdf_parser import URDFParser
    from .rviz import RvizWidget

from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import QTreeWidget, QTreeWidgetItem


class FrameTreeWidget(QTreeWidget):
    WIDTH = 200

    def __init__(self, urdf_parser: URDFParser, rviz: RvizWidget) -> None:
        super().__init__()
        self._urdf_parser = urdf_parser
        self._rviz = rviz

        self.setFixedWidth(self.WIDTH)
        self.setColumnCount(1)
        self.setHeaderLabels(["Frames Tree"])

        self.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)

        self.itemClicked.connect(self._on_item_clicked)
        self.itemExpanded.connect(self._resize_columns)
        self.itemCollapsed.connect(self._resize_columns)

    def update_internal_data_structures(self) -> None:
        # ツリーを消去
        self.clear()

        # ルートリンクから再帰的にリンクをTreeに追加していく．
        # cf. https://doc.qt.io/qtforpython/tutorials/basictutorial/treewidget.html
        root = self._urdf_parser.get_root()
        root_item = QTreeWidgetItem([root.name])
        self._add_tree_items_rec(root_item)
        self.insertTopLevelItem(0, root_item)

        self._resize_columns()

    @pyqtSlot(QTreeWidgetItem, int)
    def _on_item_clicked(self, item: QTreeWidgetItem, col: int) -> None:
        assert col == 0
        link_name = item.text(col)
        self._rviz.highlight_link(link_name)

    @pyqtSlot()
    def _resize_columns(self) -> None:
        """文字列の長さに応じて列の幅を調整する．"""
        self.resizeColumnToContents(0)

    def _add_tree_items_rec(self, parent_item: QTreeWidgetItem) -> None:
        parent_name = parent_item.text(0)

        if self._urdf_parser.is_end_link(parent_name):
            return

        for _, child_name in self._urdf_parser.get_children(parent_name):
            child_item = QTreeWidgetItem([child_name])
            parent_item.addChild(child_item)
            self._add_tree_items_rec(child_item)
