from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from typing import Union, List
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import ComboBox, SpinBox, DoubleSpinBox
from dh_rqt_tools.messages import q_error

from .base_setting import BaseSettingWidget
from ..constants import *


class RotaryWingsWidget(BaseSettingWidget):

    LABEL_PSIZE = 12

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Rotary Wings"
        abst_text = "TODO: abstruct"
        super().__init__(main, title_text, abst_text)

        links_label = QLabel("Available Links")
        links_label.setFont(QFont("Default", pointSize=self.LABEL_PSIZE, weight=QFont.Bold))
        links_label.setAlignment(Qt.AlignLeft)
        self._rows.addWidget(links_label)

        self.available = AvailableLinksWidget(main)
        self._rows.addWidget(self.available)

        self.selected = SelectedLinksWidget(main)
        self._rows.addWidget(self.selected)

        self._add_dummy_widget()

    def define_connections(self) -> None:
        super().define_connections()
        self.available.define_connections()
        self.selected.define_connections()


class AvailableLinksWidget(QListWidget):

    ITEM_HEIGHT = 40

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

    def define_connections(self) -> None:
        self._main.urdf_parser.robot_model_updated.connect(self._add_available_links)

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
                break
        else:
            raise RuntimeError(f'Link name not found: {link_name}')

    @pyqtSlot()
    def _add_available_links(self) -> None:
        """ rootから複数のfixedと1つのcontinuousで繋がったリンクのみプロペラ候補とする． """
        root_link = self._main.urdf_parser.get_root()
        links = self._main.urdf_parser.get_links()
        fixed_link_names = self._main.urdf_parser.get_fixed_link_names()

        for link in links:
            if link.name == root_link.name:
                continue

            joint = self._main.urdf_parser.get_joint(link.name)
            parent = self._main.urdf_parser.get_parent(link.name)
            if joint.type == "continuous" and parent.name in fixed_link_names:
                self.add(link.name)

        self.sortItems()

    def _link_exists_in_list(self, link_name: str) -> bool:
        items = self.findItems(link_name, Qt.MatchExactly)
        return len(items) > 0


class AvailableLinkItemWidget(QListWidget):

    BUTTON_HEIGHT = 20
    BUTTON_WIDTH = 60

    add = pyqtSignal(str)

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
        self._main.settings.rotary_wings.available.remove(self.link_name())
        self._main.settings.rotary_wings.selected.add(self.link_name())


class SelectedLinksWidget(QTabWidget):

    TAB_HEIGHT = 40
    TAB_WIDTH = 100

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self.settings: List[SelectedLinkTabWidget] = []

        self.setStyleSheet(
            f'QTabBar::tab {{ height: {self.TAB_HEIGHT}px; width: {self.TAB_WIDTH}px; }}'
        )

    def define_connections(self) -> None:
        pass

    @pyqtSlot(str)
    def add(self, link_name: str) -> None:
        setting = SelectedLinkTabWidget(link_name)
        self.settings.append(setting)
        self.addTab(setting, link_name)


class SelectedLinkTabWidget(QWidget):

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main
