from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import add_expanding_widget
from dh_rqt_tools.messages import q_error_named

from .base_setting import BaseSettingWidget
from ..parameter_getters import *


class BatteryWidget(BaseSettingWidget):

    NAME = "Battery"

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Battery"
        abst_text = "LiPoバッテリーの設定を行います．"\
            + "1つのバッテリーで全てのモータを駆動することを想定しています．"\
            + "つまり，ここでの設定は全てのモータの制御に影響します．"
        super().__init__(main, title_text, abst_text)

        voltage_description = "バッテリーの電圧．"\
            + "バッテリーが複数のセルを持つ場合，全てのセルの電圧の合計値を入力してください．"\
            + "例えば4Sの場合，通常の放電状態だと3.7[V] x 4 = 14.8[V]となります．"
        self.voltage = ParamGetterWidget_DoubleSpinBox(
            "Voltage",
            voltage_description,
            decimals=1,
            minimum=0.,
            default=14.8,
            suffix=" V",
        )
        self._rows.addWidget(self.voltage)

        capacity_description = "バッテリーが1時間に供給できる電流の量．"\
            + "例えば，5000mAhのバッテリーは1時間に5000mA (5A) の電流を供給することができます．"
        self.capacity = ParamGetterWidget_SpinBox(
            "Current Capacity",
            capacity_description,
            minimum=1,
            default=5000,
            suffix=" mAh",
        )
        # self._rows.addWidget(self.capacity)  # TODO

        C_cont_description = "バッテリーが連続的に放電できる最大の電流を示す値．"\
            + "このレートを超えてバッテリーを使用すると，過熱，パフォーマンスの低下，寿命の短縮，"\
            + "または最悪の場合，火災や爆発などの危険が発生する可能性があります．"\
            + "この値は通常，バッテリーの容量 (mAh) の倍数 (Cレートとも呼ばれます) で示されます．"\
            + "例えば，1000mAhのバッテリーが2Cの連続放電電流レートを持つ場合，"\
            + "そのバッテリーは最大2000mA (2A) の電流を連続的に供給できます．"
        self.C_cont = ParamGetterWidget_SpinBox(
            "Continuous Discharge Current Rate",
            C_cont_description,
            minimum=1,
            default=50,
            suffix=" /h",
        )
        # self._rows.addWidget(self.C_cont)  # TODO

        C_pulse_description = "バッテリーが短時間で放電できる最大電流．"\
            + "これは，バッテリーが連続的には処理できない大きな電流を一時的に供給する場合の最大レートを示します．"
        self.C_pulse = ParamGetterWidget_SpinBox(
            "Pulse Discharge Current Rate",
            C_pulse_description,
            minimum=1,
            default=100,
            suffix=" /h",
        )
        # self._rows.addWidget(self.C_pulse)  # TODO

        add_expanding_widget(self._rows)

    def define_connections(self) -> None:
        super().define_connections()

    def is_valid(self) -> bool:
        C_cont = self.C_cont.get()
        C_pulse = self.C_pulse.get()
        if C_cont > C_pulse:
            q_error_named(
                self._main,
                self.NAME,
                "Continuous discharge current rate cannot be "
                "greater than pulse discharge current rate."
            )
            return False

        return True
