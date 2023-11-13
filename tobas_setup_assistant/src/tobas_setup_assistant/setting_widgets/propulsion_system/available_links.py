from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from kdl_sympy.joint import JointType
from dh_rqt_tools.widgets import ListWidgetItem

from ...parameter_getters import *
from ...common import *


class AvailableLinksWidget(QListWidget):
    HEIGHT = 200
    ITEM_HEIGHT = 40

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()

        self._main = main

        self.setFixedHeight(self.HEIGHT)

    def define_connections(self) -> None:
        self._main.settings.propulsion_system.add_link.connect(self._remove_link)
        self._main.settings.propulsion_system.remove_link.connect(self._add_link)
        self._main.urdf_parser.robot_model_updated.connect(self._add_available_links)

    def is_valid(self) -> bool:
        return True

    @pyqtSlot()
    def _add_available_links(self) -> None:
        """
        以下の条件を満たすリンクを推進系候補としてリストに追加する．
        - continuousタイプのジョイントをもつ．
        - 回転軸が常にZ軸と一致している．
        """
        urdf_parser = self._main.urdf_parser
        root_link = urdf_parser.get_root()

        for link in urdf_parser.get_links():
            if link.name == root_link.name:
                continue

            # 無制限回転のみ
            joint = urdf_parser.get_joint(link.name)
            if joint.type != JointType.CONTINUOUS:
                continue

            # リンク名をリストに追加
            self._add_link(link.name)

        self.sortItems()

    @pyqtSlot(str)
    def _add_link(self, link_name: str) -> None:
        assert self._main.urdf_parser.link_exists(
            link_name
        ), f"Unknown link: {link_name}"
        assert not self._link_exists_in_list(link_name), f"Duplicated: {link_name}"

        item = ListWidgetItem()
        item.setSizeHint(QSize(0, self.ITEM_HEIGHT))  # 横幅が小さすぎる場合は自動で引き伸ばされる
        item.setData(Qt.UserRole, link_name)  # リンク名をソート基準にする
        self.addItem(item)
        self.setItemWidget(item, AvailableLinkItemWidget(self._main, link_name))

        # リンクが追加されるたびにソート
        self.sortItems(Qt.AscendingOrder)

    @pyqtSlot(str)
    def _remove_link(self, link_name: str) -> None:
        for row in range(self.count()):
            item = self.item(row)
            link_widget: AvailableLinkItemWidget = self.itemWidget(item)
            if link_widget.link_name() == link_name:
                self.takeItem(row)
                return
        else:
            raise RuntimeError(f"Link name not found: {link_name}")

    def _link_exists_in_list(self, link_name: str) -> bool:
        items = self.findItems(link_name, Qt.MatchExactly)
        return len(items) > 0


class AvailableLinkItemWidget(QListWidget):
    BUTTON_HEIGHT = 20
    BUTTON_WIDTH = 60

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__()

        self._main = main

        cols = QHBoxLayout()
        self.setLayout(cols)

        self._link_label = QLabel(link_name)
        self._link_label.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._link_label.setAlignment(Qt.AlignLeft)
        cols.addWidget(self._link_label)

        self._add_button = QPushButton("Add")
        self._add_button.setFixedSize(QSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT))
        cols.addWidget(self._add_button)

        self._define_connections()

    def link_name(self) -> str:
        return self._link_label.text()

    def _define_connections(self) -> None:
        self._add_button.clicked.connect(self._on_add_button_clicked)

    @pyqtSlot()
    def _on_add_button_clicked(self) -> None:
        self._main.settings.propulsion_system.add_link.emit(self.link_name())
        self._main.signals.airframe_updated.emit()
