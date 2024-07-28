from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant
    from .base import BaseHardwareWidget

from overrides import override

from tobas_rqt_tools.widgets import ComboBox, StackedWidget

from ..base_setting import BaseSettingWidget
from .navio2 import Navio2Widget


class HardwareWidget(BaseSettingWidget):
    NAME = "Hardware"
    TITLE_TEXT = "Select Flight Controller Hardware"
    ABST_TEXT = ""  # TODO

    HARDWARE_TYPE_KEY = "hardware_type"

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(main)

        self._type = ComboBox()
        self._hardwares = StackedWidget()

        self._type.currentIndexChanged.connect(self._hardwares.setCurrentIndex)

        for hardware_class in [Navio2Widget]:
            self._type.addItem(hardware_class.NAME)
            self._hardwares.addWidget(hardware_class(main))

        self._rows.addWidget(self._type)
        self._rows.addWidget(self._hardwares)
        self._rows.addStretch()

    @override
    def update_internal_data_structures(self) -> None:
        pass

    @override
    def is_valid(self) -> bool:
        if not self._selected().is_valid():
            return False

        return True

    @override
    def dump_settings(self) -> dict:
        res = dict()

        res[self.HARDWARE_TYPE_KEY] = self._type.currentText()

        for i in range(self._hardwares.count()):
            observer: BaseHardwareWidget = self._hardwares.widget(i)
            res[observer.NAME] = observer.dump_settings()

        return res

    @override
    def load_settings(self, data: dict) -> None:
        self._type.setCurrentText(data[self.HARDWARE_TYPE_KEY])

        for i in range(self._hardwares.count()):
            observer: BaseHardwareWidget = self._hardwares.widget(i)
            observer.load_settings(data[observer.NAME])

    def pkg_name(self) -> str:
        return self._selected().PACKAGE_NAME

    def _selected(self) -> BaseHardwareWidget:
        return self._hardwares.currentWidget()
