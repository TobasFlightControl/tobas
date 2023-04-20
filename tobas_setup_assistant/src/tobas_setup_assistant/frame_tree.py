from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


class FrameTreeWidget(QTreeWidget):

    WIDTH = 200

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self.setFixedWidth(self.WIDTH)
        self.setColumnCount(1)
        self.setHeaderLabels(["Frames Tree"])

    def define_connections(self) -> None:
        self.itemClicked.connect(self._on_item_clicked)
        self._main.urdf_parser.robot_model_updated.connect(self._add_tree_items)

    @pyqtSlot(QTreeWidgetItem, int)
    def _on_item_clicked(self, item: QTreeWidgetItem, col: int) -> None:
        assert col == 0
        link_name = item.text(col)
        self._main.robot_visualizer.rviz.highlight_link(link_name)

    @pyqtSlot()
    def _add_tree_items(self) -> None:
        """
        ルートリンクから再帰的にリンクをTreeに追加していく．
        cf. https://doc.qt.io/qtforpython/tutorials/basictutorial/treewidget.html
        """
        root = self._main.urdf_parser.get_root()
        root_item = QTreeWidgetItem([root.name])
        self._add_tree_items_rec(root_item)
        self.insertTopLevelItem(0, root_item)

    def _add_tree_items_rec(self, parent_item: QTreeWidgetItem) -> None:
        parent_name = parent_item.text(0)

        if self._main.urdf_parser.is_end_link(parent_name):
            return

        for _, child_name in self._main.urdf_parser.get_children(parent_name):
            child_item = QTreeWidgetItem([child_name])
            parent_item.addChild(child_item)
            self._add_tree_items_rec(child_item)
