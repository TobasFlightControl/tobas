from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from abc import abstractmethod
from overrides import overrides
from typing import List
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import ComboBox, add_expanding_widget
from dh_rqt_tools.messages import q_error_named

from .base_setting import BaseSettingWidget
from ..parameter_getters import *
from ..common import *


class BatteryWidget(BaseSettingWidget):
    NAME = "Battery"

    NO_SELECT = "Select battery type"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Battery"
        abst_text = (
            "LiPoバッテリーの設定を行います．"
            + "1つのバッテリーで全てのモータを駆動することを想定しています．"
            + "つまり，ここでの設定は全てのモータの制御に影響します．"
        )
        super().__init__(main, title_text, abst_text)

        self._batteries: List[BatteryWidget_Base] = [
            BatteryWidget_LiPo(main),
            BatteryWidget_Other(main),
        ]

        self._type = ComboBox()
        self._type.addItem(self.NO_SELECT)
        self._rows.addWidget(self._type)

        for battery in self._batteries:
            self._rows.addWidget(battery)
            self._type.addItem(battery.NAME)

        self._type.setCurrentText(BatteryWidget_LiPo.NAME)  # Default

        add_expanding_widget(self._rows)
        self._update_visibility()

    @overrides
    def define_connections(self) -> None:
        super().define_connections()
        self._type.currentTextChanged.connect(self._on_battery_type_changed)

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

    def sag_voltage(self) -> float:
        return self._selected().sag_voltage()

    def max_current(self) -> float:
        return self._selected().max_current()

    def capacity(self) -> float:
        return self._selected().capacity()

    def voltage_threshold(self) -> float:
        return self._selected().voltage_threshold()

    def _selected(self) -> BatteryWidget_Base:
        battery_type = self._type.currentText()

        if battery_type == self.NO_SELECT:
            raise RuntimeError("Battery type is not selected.")

        for battery in self._batteries:
            if battery_type == battery.NAME:
                return battery

        RuntimeError(f"Unknown battery type: {battery_type}")

    def _update_visibility(self) -> None:
        battery_type = self._type.currentText()

        for battery in self._batteries:
            battery.setVisible(False)

        for battery in self._batteries:
            if battery.NAME == battery_type:
                battery.setVisible(True)
                return

    @pyqtSlot(str)
    def _on_battery_type_changed(self, _: str) -> None:
        self._update_visibility()


class BatteryWidget_Base(QWidget):
    NAME = UNKNOWN

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
    def sag_voltage(self) -> float:
        """[V]"""
        raise NotImplementedError()

    @abstractmethod
    def max_current(self) -> float:
        """[A]"""
        raise NotImplementedError()

    @abstractmethod
    def capacity(self) -> float:
        """[As]"""
        raise NotImplementedError()

    @abstractmethod
    def voltage_threshold(self) -> float:
        """[V]"""
        raise NotImplementedError()


class BatteryWidget_LiPo(BatteryWidget_Base):
    NAME = "Lithium Polymer Battery (LiPo)"

    MAX_VOLTAGE_PER_CELL = 4.2  # 1セルあたりの最大電圧
    SAG_VOLTAGE_PER_CELL = 3.4  # 放電特性が急激に変化する電圧
    VOLTAGE_THR_PER_CELL = 3.5

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(main)

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        num_cells_description = "バッテリーのセル数．"
        self._num_cells = ParamGetterWidget_SpinBox(
            "Number of Cells",
            num_cells_description,
            minimum=1,
            maximum=8,
            default=4,
        )
        self._rows.addWidget(self._num_cells)

        capacity_description = "バッテリーから取り出すことのできる電気量．"
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
    def sag_voltage(self) -> float:
        return self._num_cells.get() * self.SAG_VOLTAGE_PER_CELL

    @overrides
    def max_current(self) -> float:
        return self._capacity.get() * self._C_cont.get() / 1000

    @overrides
    def capacity(self) -> float:
        return self._capacity.get() * 3600 / 1000

    @overrides
    def voltage_threshold(self) -> float:
        return self._num_cells.get() * self.VOLTAGE_THR_PER_CELL


class BatteryWidget_Other(BatteryWidget_Base):
    NAME = "Other Battery"

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

        sag_voltage_description = "放電特性が急激に変化する電圧．"
        self._sag_voltage = ParamGetterWidget_DoubleSpinBox(
            "Voltage Threshold",
            sag_voltage_description,
            decimals=1,
            minimum=0.1,
            default=13.6,
            suffix=" V",
        )
        self._rows.addWidget(self._sag_voltage)

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

        capacity_description = "バッテリーから取り出すことのできる電気量．"
        self._capacity = ParamGetterWidget_SpinBox(
            "Current Capacity",
            capacity_description,
            minimum=1,
            default=5000,
            suffix=" mAh",
        )
        self._rows.addWidget(self._capacity)

    @overrides
    def is_valid(self) -> bool:
        if self._max_voltage.get() <= self._sag_voltage.get():
            q_error_named(
                self._main,
                self.NAME,
                "Maximum voltage must be greater than voltage threshold.",
            )
            return False

    @overrides
    def max_voltage(self) -> float:
        return self._max_voltage.get()

    @overrides
    def sag_voltage(self) -> float:
        return self._sag_voltage.get()

    @overrides
    def max_current(self) -> float:
        return self._max_current.get()

    @overrides
    def capacity(self) -> float:
        return self._capacity.get() * 3600 / 1000

    @overrides
    def voltage_threshold(self) -> float:
        return self.sag_voltage()  # TODO: sag_voltageとは分けるべきかも
