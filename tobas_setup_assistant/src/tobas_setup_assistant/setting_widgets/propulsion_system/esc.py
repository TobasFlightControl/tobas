from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from abc import abstractmethod
from overrides import overrides
from typing import List
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import ComboBox
from dh_rqt_tools.messages import q_error_named

from ...parameter_getters import *
from ...common import *
from .common import ROTARY_WINGS


class EscWidget(QWidget):
    NO_SELECT = "Select ESC type"

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__()

        self._main = main
        self._link_name = link_name

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        title = QLabel("ESC Settings")
        title.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Bold))
        title.setAlignment(Qt.AlignTop)
        self._rows.addWidget(title)

        self._escs: List[EscWidget_Base] = [
            EscWidget_PWM(main, link_name),
            # EscWidget_DSHOT(main, link_name),
        ]

        self._type = ComboBox()
        self._type.addItem(self.NO_SELECT)
        self._rows.addWidget(self._type)

        for esc in self._escs:
            self._rows.addWidget(esc)
            self._type.addItem(esc.NAME)

        self._type.setCurrentText(EscWidget_PWM.NAME)  # Default

        self._update_visibility()
        self._define_connections()

    def is_valid(self) -> bool:
        if self._type.currentText() == self.NO_SELECT:
            q_error_named(self._main, ROTARY_WINGS, "Please select ESC type.")
            return False

        if not self._selected().is_valid():
            return False

        return True

    def copy_from(self, src: EscWidget) -> None:
        self._type.setCurrentText(src._type.currentText())

        for des_esc, src_esc in zip(self._escs, src._escs):
            des_esc.copy_from(src_esc)

        self._update_visibility()

    def esc_type(self) -> str:
        return self._type.currentText()

    def _define_connections(self) -> None:
        self._type.currentTextChanged.connect(self._on_type_changed)

    def _selected(self) -> EscWidget_Base:
        esc_type = self._type.currentText()

        if esc_type == self.NO_SELECT:
            raise RuntimeError("Observer type is not selected.")

        for esc in self._escs:
            if esc_type == esc.NAME:
                return esc

        RuntimeError(f"Unknown ESC type: {esc_type}")

    def _update_visibility(self) -> None:
        esc_type = self._type.currentText()

        for esc in self._escs:
            esc.setVisible(False)

        for esc in self._escs:
            if esc.NAME == esc_type:
                esc.setVisible(True)
                return

    @pyqtSlot(str)
    def _on_type_changed(self, _type: str) -> None:
        self._update_visibility()


class EscWidget_Base(QWidget):
    NAME = UNKNOWN

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__()

        self._main = main
        self._link_name = link_name

    @abstractmethod
    def is_valid(self) -> bool:
        raise NotImplementedError()

    @abstractmethod
    def copy_from(self, src) -> None:
        raise NotImplementedError()


class EscWidget_PWM(EscWidget_Base):
    NAME = "PWM"

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__(main, link_name)

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst_text = "周波数50Hzで1000usから2000usのパルス幅を受け取る通常のESCです．"
        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=BODY_PSIZE))
        abst.setAlignment(Qt.AlignTop)
        abst.setWordWrap(True)
        self._rows.addWidget(abst)

    @overrides
    def is_valid(self) -> bool:
        return True

    @overrides
    def copy_from(self, src: EscWidget_PWM) -> None:
        pass


class EscWidget_DSHOT(EscWidget_Base):
    NAME = "DSHOT"

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__(main, link_name)

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        abst_text = "TODO: abstraction"  # TODO
        abst = QLabel(abst_text)
        abst.setFont(QFont("Default", pointSize=BODY_PSIZE))
        abst.setAlignment(Qt.AlignTop)
        abst.setWordWrap(True)
        self._rows.addWidget(abst)

    @overrides
    def is_valid(self) -> bool:
        return True

    @overrides
    def copy_from(self, src: EscWidget_DSHOT) -> None:
        pass
