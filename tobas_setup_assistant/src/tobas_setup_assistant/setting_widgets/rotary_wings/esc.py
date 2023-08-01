from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from abc import abstractmethod
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import ComboBox
from dh_rqt_tools.messages import q_error_named

from ...parameter_getters import *
from ...constants import *
from .constants import ROTARY_WINGS


class EscWidget(QWidget):

    NO_SELECT = "Select ESC type"
    PWM = "PWM"
    DSHOT = "DSHOT"

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

        self.esc_type = ComboBox()
        self.esc_type.addItems([self.NO_SELECT, self.PWM])
        self.esc_type.setCurrentText(self.PWM)
        self._rows.addWidget(self.esc_type)

        self.pwm = EscWidget_PWM(main, link_name)
        self._rows.addWidget(self.pwm)

        self.dshot = EscWidget_DSHOT(main, link_name)
        self._rows.addWidget(self.dshot)

        self._update_visibility()
        self._define_connections()

    def is_valid(self) -> bool:
        if self.esc_type.currentText() == self.NO_SELECT:
            q_error_named(self._main, ROTARY_WINGS, "Please select ESC type.")
            return False
        
        if not self.selected().is_valid():
            return False

        return True

    def selected(self) -> EscWidget_Base:
        esc_type = self.esc_type.currentText()

        if esc_type == self.PWM:
            return self.pwm
        elif esc_type == self.DSHOT:
            return self.dshot
        else:
            raise RuntimeError()

    def copy_from(self, src: EscWidget) -> None:
        self.esc_type.setCurrentText(src.esc_type.currentText())
        self.pwm.copy_from(src.pwm)
        self.dshot.copy_from(self.dshot)

        self._update_visibility()

    def _define_connections(self) -> None:
        self.esc_type.currentTextChanged.connect(self._on_type_changed)

    def _update_visibility(self) -> None:
        esc_type = self.esc_type.currentText()

        if esc_type == self.NO_SELECT:
            self.pwm.setVisible(False)
            self.dshot.setVisible(False)
        elif esc_type == self.PWM:
            self.pwm.setVisible(True)
            self.dshot.setVisible(False)
        elif esc_type == self.DSHOT:
            self.pwm.setVisible(False)
            self.dshot.setVisible(True)
        else:
            raise RuntimeError(f'Unknown ESC type: {esc_type}')

    @pyqtSlot(str)
    def _on_type_changed(self, esc_type: str) -> None:
        self._update_visibility()


class EscWidget_Base(QWidget):

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

    def is_valid(self) -> bool:
        return True

    def copy_from(self, src: EscWidget_PWM) -> None:
        pass


class EscWidget_DSHOT(EscWidget_Base):

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

    def is_valid(self) -> bool:
        return True

    def copy_from(self, src: EscWidget_DSHOT) -> None:
        pass
