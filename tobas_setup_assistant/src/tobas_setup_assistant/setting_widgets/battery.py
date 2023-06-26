from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from abc import abstractmethod
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import ComboBox, add_expanding_widget
from dh_rqt_tools.messages import q_error_named

from .base_setting import BaseSettingWidget
from ..parameter_getters import *


class BatteryWidget(BaseSettingWidget):

    NAME = "Battery"

    NO_SELECT = "Select battery type"
    LIPO = "Lithium Polymer Battery (LiPo)"
    OTHER = "Other Battery"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Battery"
        abst_text = "LiPoバッテリーの設定を行います．"\
            + "1つのバッテリーで全てのモータを駆動することを想定しています．"\
            + "つまり，ここでの設定は全てのモータの制御に影響します．"
        super().__init__(main, title_text, abst_text)

        self.battery_type = ComboBox()
        self.battery_type.addItems([self.NO_SELECT, self.LIPO, self.OTHER])
        self.battery_type.setCurrentText(self.NO_SELECT)
        self._rows.addWidget(self.battery_type)

        self.lipo = BatteryWidget_LiPo(main)
        self._rows.addWidget(self.lipo)

        self.other = BatteryWidget_Other(main)
        self._rows.addWidget(self.other)

        add_expanding_widget(self._rows)
        self._update_visibility()

    def define_connections(self) -> None:
        super().define_connections()
        self.battery_type.currentTextChanged.connect(self._on_type_changed)

    def is_valid(self) -> bool:
        if self.battery_type.currentText() == self.NO_SELECT:
            q_error_named(self._main, self.NAME, "Please select battery type.")
            return False

        if not self.selected().is_valid():
            return False

        return True

    def selected(self) -> BatteryWidget_Base:
        battery_type = self.battery_type.currentText()

        if battery_type == self.LIPO:
            return self.lipo
        elif battery_type == self.OTHER:
            return self.other
        else:
            raise RuntimeError(f'Invalid battery type: {battery_type}')

    def _update_visibility(self) -> None:
        battery_type = self.battery_type.currentText()

        if battery_type == self.NO_SELECT:
            self.lipo.setVisible(False)
            self.other.setVisible(False)
        elif battery_type == self.LIPO:
            self.lipo.setVisible(True)
            self.other.setVisible(False)
        elif battery_type == self.OTHER:
            self.lipo.setVisible(False)
            self.other.setVisible(True)
        else:
            raise RuntimeError(f'Invalid battery type: {battery_type}')

    @pyqtSlot(str)
    def _on_type_changed(self, battery_type: str) -> None:
        self._update_visibility()


class BatteryWidget_Base(QWidget):

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()

        self._main = main

    @abstractmethod
    def is_valid(self) -> bool:
        raise NotImplementedError()

    @abstractmethod
    def nominal_voltage(self) -> float:
        raise NotImplementedError()


class BatteryWidget_LiPo(BatteryWidget_Base):

    VOLTAGE_PER_CELL = 3.7  # 1セルあたりの定格電圧

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(main)

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        num_cells_description = "セル数．1セルあたりの定格電圧は3.7V．"
        self._num_cells = ParamGetterWidget_SpinBox(
            "Number of cells",
            num_cells_description,
            minimum=1,
            maximum=8,
            default=4,
        )
        self._rows.addWidget(self._num_cells)

        capacity_description = "バッテリーが1時間に供給できる電流の量．"\
            + "例えば，5000mAhのバッテリーは1時間に5000mA (5A) の電流を供給することができます．"
        self._capacity = ParamGetterWidget_SpinBox(
            "Current Capacity",
            capacity_description,
            minimum=1,
            default=5000,
            suffix=" mAh",
        )
        # self._rows.addWidget(self._capacity)  # TODO

        C_cont_description = "バッテリーが連続的に放電できる最大の電流を示す値．"\
            + "このレートを超えてバッテリーを使用すると，過熱，パフォーマンスの低下，寿命の短縮，"\
            + "または最悪の場合，火災や爆発などの危険が発生する可能性があります．"\
            + "この値は通常，バッテリーの容量 (mAh) の倍数 (Cレートとも呼ばれます) で示されます．"\
            + "例えば，1000mAhのバッテリーが2Cの連続放電電流レートを持つ場合，"\
            + "そのバッテリーは最大2000mA (2A) の電流を連続的に供給できます．"
        self._C_cont = ParamGetterWidget_SpinBox(
            "Continuous Discharge Current Rate",
            C_cont_description,
            minimum=1,
            default=50,
            suffix=" /h",
        )
        # self._rows.addWidget(self._C_cont)  # TODO

        C_pulse_description = "バッテリーが短時間で放電できる最大電流．"\
            + "これは，バッテリーが連続的には処理できない大きな電流を一時的に供給する場合の最大レートを示します．"
        self._C_pulse = ParamGetterWidget_SpinBox(
            "Pulse Discharge Current Rate",
            C_pulse_description,
            minimum=1,
            default=100,
            suffix=" /h",
        )
        # self._rows.addWidget(self._C_pulse)  # TODO

    def is_valid(self) -> bool:
        C_cont = self._C_cont.get()
        C_pulse = self._C_pulse.get()
        if C_cont > C_pulse:
            q_error_named(
                self._main,
                self._main.settings.battery.LIPO,
                "Continuous discharge current rate cannot be "
                "greater than pulse discharge current rate."
            )
            return False

        return True

    def nominal_voltage(self) -> float:
        return self._num_cells.get() * self.VOLTAGE_PER_CELL


class BatteryWidget_Other(BatteryWidget_Base):

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(main)

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        nominal_voltage_description = "バッテリーの定格電圧．"
        self._nominal_voltage = ParamGetterWidget_DoubleSpinBox(
            "Nominal Voltage",
            nominal_voltage_description,
            decimals=1,
            minimum=0.1,
            default=14.8,
            suffix=" V",
        )
        self._rows.addWidget(self._nominal_voltage)

    def is_valid(self) -> bool:
        return True

    def nominal_voltage(self) -> float:
        return self._nominal_voltage.get()
