from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant
    from ..parameter_getters import ParamGetterWidget

from abc import abstractmethod
from overrides import override
from typing import final
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import QWidget, QLabel, QCheckBox, QVBoxLayout
from PyQt5.QtGui import QFont

from tobas_rqt_tools.widgets import ScrollArea

from ..common import TITLE_PSIZE, BODY_PSIZE, TO_DO, Description


class BaseSettingWidget(ScrollArea):
    NAME = TO_DO
    TITLE_TEXT = TO_DO
    ABST_TEXT = TO_DO

    ABST_HEIGHT = 100

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        title = QLabel(self.TITLE_TEXT)
        title.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Bold))
        title.setAlignment(Qt.AlignTop)
        self._rows.addWidget(title)

        abst = Description(self.ABST_TEXT)
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

    @abstractmethod
    def dump_settings(self) -> dict:
        """ユーザ設定を記録した辞書を返す．"""
        raise NotImplementedError()

    @abstractmethod
    def load_settings(self, data: dict) -> None:
        """ユーザ設定を読み込む．"""
        raise NotImplementedError()

    @abstractmethod
    def on_opened(self) -> None:
        """タブが開かれた時に呼ばれるコールバック．表示内容が他のタブの状態に依存する場合に使う．"""
        pass


class OptionalDeviceWidget(BaseSettingWidget):
    """
    オプションデバイスの共通機能を備えた設定ウィジェット．\\
    搭載する場合のみ各種設定項目が有効になる．
    """

    def __init__(self, main: SetupAssistant, default_equipped: bool) -> None:
        super().__init__(main)

        self._equipped = QCheckBox(f"{self.NAME} Equipped")
        self._equipped.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._equipped.setChecked(default_equipped)
        self._equipped.toggled.connect(self._on_equipped_toggled)
        self._rows.addWidget(self._equipped)

        # Enable, Disableを一括で管理するために，設定ウィジェットを全て1つのウィジェットの子にする．
        self._config = QWidget()
        self._config.setEnabled(default_equipped)
        self._rows.addWidget(self._config)

        self._param_rows = QVBoxLayout()
        self._config.setLayout(self._param_rows)

    @override
    def dump_settings(self) -> dict:
        """_add_param_widgetでParameterGetterWidgetを追加しただけならば有効．"""
        res = dict()

        res[self._equipped.text()] = self._equipped.isChecked()

        for i in range(self._param_rows.count()):
            param: ParamGetterWidget = self._param_rows.itemAt(i).widget()
            res[param.name()] = param.get()

        return res

    @override
    def load_settings(self, data: dict) -> None:
        """_add_param_widgetでParameterGetterWidgetを追加しただけならば有効．"""
        self._equipped.setChecked(data[self._equipped.text()])

        for i in range(self._param_rows.count()):
            param: ParamGetterWidget = self._param_rows.itemAt(i).widget()
            param.set(data[param.name()])

    @final
    def equipped(self) -> bool:
        return self._equipped.isChecked()

    @final
    def _add_param_widget(self, widget: ParamGetterWidget) -> None:
        """Equippedがチェックされているときだけ有効になるウィジェットを追加する．"""
        self._param_rows.addWidget(widget)

    @pyqtSlot(bool)
    def _on_equipped_toggled(self, checked: bool) -> None:
        self._config.setEnabled(checked)
