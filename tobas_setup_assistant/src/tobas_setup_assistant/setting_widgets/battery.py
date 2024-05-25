from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from abc import abstractmethod
from overrides import override
from PyQt5.QtWidgets import QVBoxLayout

from tobas_rqt_tools.widgets import Widget, ComboBox, StackedWidget
from tobas_rqt_tools.messages import q_error_named

from ..common import TO_DO
from ..parameter_getters import ParamGetterWidget_SpinBox, ParamGetterWidget_DoubleSpinBox
from .base_setting import BaseSettingWidget


class BatteryWidget(BaseSettingWidget):
    NAME = "Battery"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Battery"
        abst_text = (
            "Configure the LiPo (Lithium Polymer) battery settings. "
            "It is assumed that a single battery will power all motors. "
            "Therefore, the settings here will affect the control of all motors."
        )
        super().__init__(main, title_text, abst_text)

        battery_types = ComboBox()
        self._batteries = StackedWidget()
        battery_types.currentIndexChanged.connect(self._batteries.setCurrentIndex)
        self._rows.addWidget(battery_types)
        self._rows.addWidget(self._batteries)

        for battery_class in [BatteryWidget_LiPo, BatteryWidget_Other]:
            battery_types.addItem(battery_class.NAME)
            self._batteries.addWidget(battery_class(main))

        # NOTE: QStackedWidgetは各子ウィジェットのサイズを合わせるため，ストレッチを各子ウィジェットで設定しないとレイアウトが崩れる．

    @override
    def define_connections(self) -> None:
        super().define_connections()

    @override
    def is_valid(self) -> bool:
        if not self._selected().is_valid():
            return False

        return True

    def nominal_voltage(self) -> float:
        return self._selected().nominal_voltage()

    def max_voltage(self) -> float:
        return self._selected().max_voltage()

    def sag_voltage(self) -> float:
        return self._selected().sag_voltage()

    def max_current(self) -> float:
        return self._selected().max_current()

    def capacity(self) -> float:
        return self._selected().capacity()

    def internal_registance(self) -> float:
        return self._selected().internal_registance()

    def _selected(self) -> BatteryWidget_Base:
        return self._batteries.currentWidget()


class BatteryWidget_Base(Widget):
    NAME = TO_DO

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

    @abstractmethod
    def is_valid(self) -> bool:
        raise NotImplementedError()

    @abstractmethod
    def nominal_voltage(self) -> float:
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
    def internal_registance(self) -> float:
        """[Ω] 内部抵抗値．"""
        raise NotImplementedError()


class BatteryWidget_LiPo(BatteryWidget_Base):
    NAME = "Lithium Polymer Battery (LiPo)"

    NOMINAL_VOLTAGE_PER_CELL = 3.7  # 1セルあたりの定格電圧
    MAX_VOLTAGE_PER_CELL = 4.2  # 1セルあたりの最大電圧
    SAG_VOLTAGE_PER_CELL = 3.4  # 放電特性が急激に変化する電圧
    VOLTAGE_THR_PER_CELL = 3.2  # 内部抵抗による降圧を考慮した警告の閾値

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(main)

        rows = QVBoxLayout()
        self.setLayout(rows)

        num_cells_description = "The number of cells in the battery."
        self._num_cells = ParamGetterWidget_SpinBox(
            "Number of Cells", num_cells_description, minimum=1, maximum=100, default=4
        )
        rows.addWidget(self._num_cells)

        capacity_description = "The amount of electric charge that can be drawn from the battery."
        self._capacity = ParamGetterWidget_SpinBox(
            "Current Capacity", capacity_description, minimum=1, default=5000, suffix=" mAh"
        )
        rows.addWidget(self._capacity)

        C_cont_description = (
            "The maximum continuous discharge current that the battery can provide. "
            "Exceeding this rate during use can lead to overheating, reduced performance, shortened lifespan, "
            "or, in the worst case, hazards such as fire or explosion. "
            "This value is typically represented as a multiple of the battery's capacity (in mAh), known as the C-rate. "
            "For example, if a 1000mAh battery has a 2C continuous discharge rate, "
            "it can continuously supply up to 2000mA (2A) of current."
        )
        self._C_cont = ParamGetterWidget_SpinBox(
            "Continuous Discharge Current Rate", C_cont_description, minimum=1, default=50, suffix=" /h"
        )
        rows.addWidget(self._C_cont)

        registance_description = "Internal resistance value per cell."
        self._registance = ParamGetterWidget_SpinBox(
            "Internal Registance", registance_description, minimum=0, default=3, suffix=" mΩ"
        )
        rows.addWidget(self._registance)

        rows.addStretch()

    @override
    def is_valid(self) -> bool:
        return True

    @override
    def nominal_voltage(self) -> float:
        return self._num_cells.get() * self.NOMINAL_VOLTAGE_PER_CELL

    @override
    def max_voltage(self) -> float:
        return self._num_cells.get() * self.MAX_VOLTAGE_PER_CELL

    @override
    def sag_voltage(self) -> float:
        return self._num_cells.get() * self.SAG_VOLTAGE_PER_CELL

    @override
    def max_current(self) -> float:
        return self._capacity.get() * self._C_cont.get() / 1000

    @override
    def capacity(self) -> float:
        return self._capacity.get() * 3600 / 1000

    @override
    def internal_registance(self) -> float:
        return self._num_cells.get() * self._registance.get() / 1000


class BatteryWidget_Other(BatteryWidget_Base):
    NAME = "Other Battery"

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(main)

        rows = QVBoxLayout()
        self.setLayout(rows)

        nominal_voltage_description = "Nominal voltage of the battery."
        self._nominal_voltage = ParamGetterWidget_DoubleSpinBox(
            "Nominal Voltage", nominal_voltage_description, decimals=1, minimum=0.1, default=14.8, suffix=" V"
        )
        rows.addWidget(self._nominal_voltage)

        max_voltage_description = "Maximum voltage of the battery."
        self._max_voltage = ParamGetterWidget_DoubleSpinBox(
            "Maximum Voltage", max_voltage_description, decimals=1, minimum=0.1, default=16.8, suffix=" V"
        )
        rows.addWidget(self._max_voltage)

        sag_voltage_description = "Voltage at which the discharge characteristics change abruptly."
        self._sag_voltage = ParamGetterWidget_DoubleSpinBox(
            "Voltage Threshold", sag_voltage_description, decimals=1, minimum=0.1, default=13.6, suffix=" V"
        )
        rows.addWidget(self._sag_voltage)

        max_current_description = "Maximum current of the battery."
        self._max_current = ParamGetterWidget_DoubleSpinBox(
            "Maximum Current", max_current_description, decimals=1, minimum=0.1, default=200.0, suffix=" A"
        )
        rows.addWidget(self._max_current)

        capacity_description = "The amount of electric charge that can be drawn from the battery."
        self._capacity = ParamGetterWidget_SpinBox(
            "Current Capacity", capacity_description, minimum=1, default=5000, suffix=" mAh"
        )
        rows.addWidget(self._capacity)

        registance_description = "Internal resistance value of the battery."
        self._registance = ParamGetterWidget_SpinBox(
            "Internal Registance", registance_description, minimum=0, default=12, suffix=" mΩ"
        )
        rows.addWidget(self._registance)

        rows.addStretch()

    @override
    def is_valid(self) -> bool:
        if self._max_voltage.get() <= self._sag_voltage.get():
            q_error_named(self._main, self.NAME, "Maximum voltage must be greater than voltage threshold.")
            return False

        return True

    @override
    def nominal_voltage(self) -> float:
        return self._nominal_voltage.get()

    @override
    def max_voltage(self) -> float:
        return self._max_voltage.get()

    @override
    def sag_voltage(self) -> float:
        return self._sag_voltage.get()

    @override
    def max_current(self) -> float:
        return self._max_current.get()

    @override
    def capacity(self) -> float:
        return self._capacity.get() * 3600 / 1000

    @override
    def internal_registance(self) -> float:
        return self._registance.get() / 1000
