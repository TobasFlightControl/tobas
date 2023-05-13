from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from ...parameter_getters import *
from ...constants import *


class AvailableLinksWidget(QListWidget):

    HEIGHT = 200
    ITEM_HEIGHT = 40

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()

        self._main = main

        self.setFixedHeight(self.HEIGHT)

    def define_connections(self) -> None:
        self._main.urdf_parser.robot_model_updated.connect(self._add_available_links)

    def is_valid(self) -> bool:
        return True

    def add(self, link_name: str) -> None:
        assert self._main.urdf_parser.link_exists(link_name), f'Unknown link: {link_name}'
        assert not self._link_exists_in_list(link_name), f'Duplicated: {link_name}'

        item = QListWidgetItem()
        item.setSizeHint(QSize(0, self.ITEM_HEIGHT))  # 横幅が小さすぎる場合は自動で引き伸ばされる
        self.addItem(item)
        self.setItemWidget(item, AvailableLinkItemWidget(self._main, link_name))

    @pyqtSlot(str)
    def remove(self, link_name: str) -> None:
        for row in range(self.count()):
            item = self.item(row)
            link_widget: AvailableLinkItemWidget = self.itemWidget(item)
            if link_widget.link_name() == link_name:
                self.takeItem(row)
                return
        else:
            raise RuntimeError(f'Link name not found: {link_name}')

    @pyqtSlot()
    def _add_available_links(self) -> None:
        """
        以下の条件を満たすリンクをプロペラ候補としてリストに追加する．
        - 親リンクがNWU-Fixed．
        - continuousタイプのジョイントをもつ．
        - 回転軸がZ軸と一致している．
        """
        root_link = self._main.urdf_parser.get_root()
        fixed_link_names = self._main.urdf_parser.nwu_fixed_link_names()

        for link in self._main.urdf_parser.get_links():
            if link.name == root_link.name:
                continue

            joint = self._main.urdf_parser.get_joint(link.name)
            parent = self._main.urdf_parser.get_parent(link.name)
            if (
                parent.name in fixed_link_names and
                joint.type == "continuous" and
                joint.axis == [0, 0, 1]
            ):
                self.add(link.name)

        self.sortItems()

    def _link_exists_in_list(self, link_name: str) -> bool:
        items = self.findItems(link_name, Qt.MatchExactly)
        return len(items) > 0


class AvailableLinkItemWidget(QListWidget):

    BUTTON_HEIGHT = 20
    BUTTON_WIDTH = 60

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__()

        self._main = main

        self._cols = QHBoxLayout()
        self.setLayout(self._cols)

        self._link_label = QLabel(link_name)
        self._link_label.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._link_label.setAlignment(Qt.AlignLeft)
        self._cols.addWidget(self._link_label)

        self._add_button = QPushButton("Add")
        self._add_button.setFixedSize(QSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT))
        self._cols.addWidget(self._add_button)

        self._define_connections()

    def link_name(self) -> str:
        return self._link_label.text()

    def _define_connections(self) -> None:
        self._add_button.clicked.connect(self._on_add_button_clicked)

    @pyqtSlot()
    def _on_add_button_clicked(self) -> None:
        self._main.settings.rotary_wings.selected.add(self.link_name())
        self._main.settings.rotary_wings.available.remove(self.link_name())
