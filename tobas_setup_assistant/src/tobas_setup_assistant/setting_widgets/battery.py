from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from abc import abstractmethod
from overrides import overrides
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
        abst_text = (
            "LiPoバッテリーの設定を行います．"
            + "1つのバッテリーで全てのモータを駆動することを想定しています．"
            + "つまり，ここでの設定は全てのモータの制御に影響します．"
        )
        super().__init__(main, title_text, abst_text)

        self._type = ComboBox()
        self._type.addItems([self.NO_SELECT, self.LIPO, self.OTHER])
        self._type.setCurrentText(self.LIPO)
        self._rows.addWidget(self._type)

        self._lipo = BatteryWidget_LiPo(main)
        self._rows.addWidget(self._lipo)

        self._other = BatteryWidget_Other(main)
        self._rows.addWidget(self._other)

        add_expanding_widget(self._rows)
        self._update_visibility()

    @overrides
    def define_connections(self) -> None:
        super().define_connections()
        self._type.currentTextChanged.connect(self._on_battery_typechanged)

    @overrides
    def is_valid(self) -> bool:
        if self._type.currentText() == self.NO_SELECT:
            q_error_named(self._main, self.NAME, "Please select battery type.")
            return False

        if not self._selected().is_valid():
            return False

        return True

    def max_voltage(self) -> float:
        return self._selected().max_voltage()

    def max_current(self) -> float:
        return self._selected().max_current()

    def voltage_threshold(self) -> float:
        return self._selected().voltage_threshold()

    def _selected(self) -> BatteryWidget_Base:
        battery_type = self._type.currentText()

        if battery_type == self.LIPO:
            return self._lipo
        elif battery_type == self.OTHER:
            return self._other
        else:
            raise RuntimeError(f"Invalid battery type: {battery_type}")

    def _update_visibility(self) -> None:
        battery_type = self._type.currentText()

        if battery_type == self.NO_SELECT:
            self._lipo.setVisible(False)
            self._other.setVisible(False)
        elif battery_type == self.LIPO:
            self._lipo.setVisible(True)
            self._other.setVisible(False)
        elif battery_type == self.OTHER:
            self._lipo.setVisible(False)
            self._other.setVisible(True)
        else:
            raise RuntimeError(f"Invalid battery type: {battery_type}")

    @pyqtSlot(str)
    def _on_battery_typechanged(self, _: str) -> None:
        self._update_visibility()


class BatteryWidget_Base(QWidget):
    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()

        self._main = main

    @abstractmethod
    def is_valid(self) -> bool:
        raise NotImplementedError()

    @abstractmethod
    def max_voltage(self) -> float:
        """[V]"""
        raise NotImplementedError()

    @abstractmethod
    def max_current(self) -> float:
        """[A]"""
        raise NotImplementedError()

    @abstractmethod
    def voltage_threshold(self) -> float:
        """[V]"""
        raise NotImplementedError()


class BatteryWidget_LiPo(BatteryWidget_Base):
    MAX_VOLTAGE_PER_CELL = 4.2  # 1セルあたりの最大電圧
    VOLTAGE_THR_PER_CELL = 3.5

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(main)

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        num_cells_description = "セル数．1セルあたりの定格電圧は3.7V．"
        self._num_cells = ParamGetterWidget_SpinBox(
            "Number of Cells",
            num_cells_description,
            minimum=1,
            maximum=8,
            default=4,
        )
        self._rows.addWidget(self._num_cells)

        capacity_description = (
            "バッテリーが1時間に供給できる電流の量．"
            + "例えば，5000mAhのバッテリーは1時間に5000mA (5A) の電流を供給することができます．"
        )
        self._capacity = ParamGetterWidget_SpinBox(
            "Current Capacity",
            capacity_description,
            minimum=1,
            default=5000,
            suffix=" mAh",
        )
        self._rows.addWidget(self._capacity)

        C_cont_description = (
            "バッテリーが連続的に放電できる最大の電流を示す値．"
            + "このレートを超えてバッテリーを使用すると，過熱，パフォーマンスの低下，寿命の短縮，"
            + "または最悪の場合，火災や爆発などの危険が発生する可能性があります．"
            + "この値は通常，バッテリーの容量 (mAh) の倍数 (Cレートとも呼ばれます) で示されます．"
            + "例えば，1000mAhのバッテリーが2Cの連続放電電流レートを持つ場合，"
            + "そのバッテリーは最大2000mA (2A) の電流を連続的に供給できます．"
        )
        self._C_cont = ParamGetterWidget_SpinBox(
            "Continuous Discharge Current Rate",
            C_cont_description,
            minimum=1,
            default=50,
            suffix=" /h",
        )
        self._rows.addWidget(self._C_cont)

    @overrides
    def is_valid(self) -> bool:
        return True

    @overrides
    def max_voltage(self) -> float:
        return self._num_cells.get() * self.MAX_VOLTAGE_PER_CELL

    @overrides
    def max_current(self) -> float:
        return self._capacity.get() * self._C_cont.get() / 1000

    @overrides
    def voltage_threshold(self) -> float:
        return self._num_cells.get() * self.VOLTAGE_THR_PER_CELL


class BatteryWidget_Other(BatteryWidget_Base):
    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(main)

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        max_voltage_description = "バッテリーの最大電圧．"
        self._max_voltage = ParamGetterWidget_DoubleSpinBox(
            "Maximum Voltage",
            max_voltage_description,
            decimals=1,
            minimum=0.1,
            default=16.8,
            suffix=" V",
        )
        self._rows.addWidget(self._max_voltage)

        max_current_description = "バッテリーの最大電流．"
        self._max_current = ParamGetterWidget_DoubleSpinBox(
            "Maximum Current",
            max_current_description,
            decimals=1,
            minimum=0.1,
            default=250.0,
            suffix=" A",
        )
        self._rows.addWidget(self._max_voltage)

        voltage_threshold_description = "安全に飛行できるバッテリー電圧の下限．"
        self._voltage_threshold = ParamGetterWidget_DoubleSpinBox(
            "Voltage Threshold",
            voltage_threshold_description,
            decimals=1,
            minimum=0.1,
            default=14.0,
            suffix=" V",
        )
        self._rows.addWidget(self._voltage_threshold)

    @overrides
    def is_valid(self) -> bool:
        if self._max_voltage.get() <= self._voltage_threshold.get():
            q_error_named(
                self._main,
                self._main.settings.battery.OTHER,
                "Maximum voltage must be greater than voltage threshold.",
            )
            return False

    @overrides
    def max_voltage(self) -> float:
        return self._max_voltage.get()

    @overrides
    def max_current(self) -> float:
        return self._max_current.get()

    @overrides
    def voltage_threshold(self) -> float:
        return self._voltage_threshold.get()
