from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from overrides import override

from tobas_rqt_tools.widgets import ComboBox, StackedWidget

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

        self._type = ComboBox()
        self._observers = StackedWidget()

        self._type.currentIndexChanged.connect(self._observers.setCurrentIndex)

        for observer_class in [ErrorStateKalmanFilter, CustomObserver]:
            self._type.addItem(observer_class.NAME)
            self._observers.addWidget(observer_class(main))

        self._rows.addWidget(self._type)
        self._rows.addWidget(self._observers)
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

        res[self.OBSERVER_TYPE] = self._type.currentText()

        for i in range(self._observers.count()):
            observer: BaseObserver = self._observers.widget(i)
            res[observer.NAME] = observer.dump_settings()

        return res

    @override
    def load_settings(self, data: dict) -> None:
        self._type.setCurrentText(data[self.OBSERVER_TYPE])

        for i in range(self._observers.count()):
            observer: BaseObserver = self._observers.widget(i)
            observer.load_settings(data[observer.NAME])

    def pkg_name(self) -> str:
        return self._selected().PACKAGE_NAME

    def static_parameters(self) -> dict:
        return self._selected().static_parameters()

    def _selected(self) -> BaseObserver:
        return self._observers.currentWidget()
