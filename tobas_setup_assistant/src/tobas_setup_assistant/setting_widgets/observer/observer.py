from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from typing import List
from overrides import override
from PyQt5.QtCore import pyqtSlot

from tobas_rqt_tools.widgets import ComboBox

from ..base_setting import BaseSettingWidget
from .base import BaseObserver
from .eskf import ErrorStateKalmanFilter
from .custom import CustomObserver


class ObserverWidget(BaseSettingWidget):
    NAME = "Observer"

    OBSERVER_TYPE = "observer_type"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Setup Observer"
        abst_text = (
            "Configure the state estimator by selecting one method and setting its parameters. "
            "You can tune the parameters later, so it's fine to leave them at their default values if preferred."
        )
        super().__init__(main, title_text, abst_text)

        self._observers: List[BaseObserver] = [ErrorStateKalmanFilter(main), CustomObserver(main)]

        self._type = ComboBox()
        self._type.currentTextChanged.connect(self._on_type_changed)
        self._rows.addWidget(self._type)

        for observer in self._observers:
            self._rows.addWidget(observer)
            self._type.addItem(observer.NAME)

        self._rows.addStretch()
        self._update_visibility()

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

        res[self.OBSERVER_TYPE] = self._type.currentText()

        for observer in self._observers:
            res[observer.NAME] = observer.dump_settings()

        return res

    @override
    def load_settings(self, data: dict) -> None:
        self._type.setCurrentText(data[self.OBSERVER_TYPE])

        for observer in self._observers:
            observer.load_settings(data[observer.NAME])

    def pkg_name(self) -> str:
        return self._selected().PACKAGE_NAME

    def static_parameters(self) -> dict:
        return self._selected().static_parameters()

    def _selected(self) -> BaseObserver:
        observer_type = self._type.currentText()

        for observer in self._observers:
            if observer_type == observer.NAME:
                return observer

        RuntimeError(f"Unknown observer type: {observer_type}")

    def _update_visibility(self) -> None:
        observer_type = self._type.currentText()

        for observer in self._observers:
            observer.setVisible(False)

        for observer in self._observers:
            if observer.NAME == observer_type:
                observer.setVisible(True)
                return

    @pyqtSlot(str)
    def _on_type_changed(self, _: str) -> None:
        self._update_visibility()
