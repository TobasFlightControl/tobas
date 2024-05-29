from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from abc import abstractmethod
from typing import final
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import QWidget, QLabel, QCheckBox, QVBoxLayout
from PyQt5.QtGui import QFont

from tobas_rqt_tools.widgets import ScrollArea

from ..common import TITLE_PSIZE, BODY_PSIZE, TO_DO, Description


class BaseSettingWidget(ScrollArea):
    ABST_HEIGHT = 100

    NAME = TO_DO

    def __init__(self, main: SetupAssistant, title_text: str, abst_text: str) -> None:
        super().__init__()
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        title = QLabel(title_text)
        title.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Bold))
        title.setAlignment(Qt.AlignTop)
        self._rows.addWidget(title)

        abst = Description(abst_text)
        abst.setFixedHeight(self.ABST_HEIGHT)
        self._rows.addWidget(abst)

    @abstractmethod
    def update_internal_data_structures(self) -> None:
        """URDFの変化に合わせて内部状態を更新する．"""
        raise NotImplementedError()

    @abstractmethod
    def is_valid(self) -> bool:
        """ユーザ設定に問題がない場合にTrueを返す．"""
        raise NotImplementedError()


class OptionalDeviceWidget(BaseSettingWidget):
    """
    オプションデバイスの共通機能を備えた設定ウィジェット．\\
    搭載する場合のみ各種設定項目が有効になる．
    """

    def __init__(self, main: SetupAssistant, title_text: str, abst_text: str, default_equipped: bool) -> None:
        super().__init__(main, title_text, abst_text)

        self._equipped = QCheckBox(f"{self.NAME} Equipped")
        self._equipped.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._equipped.setChecked(default_equipped)
        self._equipped.toggled.connect(self._on_equipped_toggled)
        self._rows.addWidget(self._equipped)

        # Enable, Disableを一括で管理するために，設定ウィジェットを全て1つのウィジェットの子にする．
        self._config = QWidget()
        self._config.setEnabled(default_equipped)
        self._rows.addWidget(self._config)

        self._config_rows = QVBoxLayout()
        self._config.setLayout(self._config_rows)

    @final
    def equipped(self) -> bool:
        return self._equipped.isChecked()

    @final
    def _add_config_widget(self, widget: QWidget) -> None:
        """Equippedがチェックされているときだけ有効になるウィジェットを追加する．"""
        self._config_rows.addWidget(widget)

    @pyqtSlot(bool)
    def _on_equipped_toggled(self, checked: bool) -> None:
        self._config.setEnabled(checked)
