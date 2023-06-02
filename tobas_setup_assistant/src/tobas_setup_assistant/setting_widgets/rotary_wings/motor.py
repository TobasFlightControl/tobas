from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant
    from .esc import EscWidget_Base

from abc import abstractmethod
from typing import final
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import ComboBox
from dh_rqt_tools.messages import q_error_named

from ...parameter_getters import *
from ...constants import *
from ...utils import remap
from .constants import NAME


class MotorWidget(QWidget):

    NO_SELECT = "Select setting method"
    MANUAL = "Set manually"
    EXPERIMENT = "Set from experimental data (recommended)"

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__()

        self._main = main
        self._link_name = link_name

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        title = QLabel("Motor Settings")
        title.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Bold))
        title.setAlignment(Qt.AlignTop)
        self._rows.addWidget(title)

        self.setting_method = ComboBox()
        self.setting_method.addItems([self.NO_SELECT, self.MANUAL,  self.EXPERIMENT])
        self.setting_method.setCurrentText(self.NO_SELECT)
        self._rows.addWidget(self.setting_method)

        self.manual = MotorWidget_Manual(main, link_name)
        self._rows.addWidget(self.manual)

        self.experiment = MotorWidget_Experiment(main, link_name)
        self._rows.addWidget(self.experiment)

        self._update_visibility()
        self._define_connections()

    def is_valid(self) -> bool:
        if self.setting_method.currentText() == self.NO_SELECT:
            print(self.setting_method.currentText())
            q_error_named(self._main, NAME, "Please select motor setting method.")
            return False
        else:
            if not self.selected().is_valid():
                return False

        return True

    def selected(self) -> EscWidget_Base:
        setting_method = self.setting_method.currentText()

        if setting_method == self.MANUAL:
            return self.manual
        elif setting_method == self.EXPERIMENT:
            return self.experiment
        else:
            raise RuntimeError

    def direction(self) -> str:
        """ 'cw' or 'ccw' """
        return self._selected_setting_widget().direction()

    def kv(self) -> float:
        """ [rpm/V], including efficiency """
        return self._selected_setting_widget().kv()

    def time_const_up(self) -> float:
        """ [s] """
        return self._selected_setting_widget().time_const_up()

    def time_const_down(self) -> float:
        """ [s] """
        return self._selected_setting_widget().time_const_down()

    def copy_from(self, src: MotorWidget) -> None:
        self.setting_method.setCurrentText(src.setting_method.currentText())
        self.manual.copy_from(src.manual)
        self.experiment.copy_from(src.experiment)

        self._update_visibility()

    def _define_connections(self) -> None:
        self.setting_method.currentTextChanged.connect(self._on_type_changed)

    def _update_visibility(self) -> None:
        setting_method = self.setting_method.currentText()

        if setting_method == self.NO_SELECT:
            self.manual.setVisible(False)
            self.experiment.setVisible(False)
        elif setting_method == self.MANUAL:
            self.manual.setVisible(True)
            self.experiment.setVisible(False)
        elif setting_method == self.EXPERIMENT:
            self.manual.setVisible(False)
            self.experiment.setVisible(True)
        else:
            raise RuntimeError(f'Unknown setting method: {setting_method}')

    def _selected_setting_widget(self) -> MotorWidget_Base:
        setting_method = self.setting_method.currentText()

        if setting_method == self.MANUAL:
            return self.manual
        elif setting_method == self.EXPERIMENT:
            return self.experiment
        else:
            raise RuntimeError(f'Unknown setting method: {setting_method}')

    @pyqtSlot(str)
    def _on_type_changed(self, setting_method: str) -> None:
        self._update_visibility()


class MotorWidget_Base(QWidget):  # ABCを継承するとバグる

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__()

        self._main = main
        self._link_name = link_name

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        direction_description = "モータの回転方向．"\
            + "X軸またはZ軸の正方向に対してCW (Clock Wise) またはCCW (Counter Clock Wise) を選択してください．"
        self._direction = ParamGetterWidget_ComboBox(
            "Rotating Direction",
            direction_description,
            ["CW", "CCW"],
        )
        self._rows.addWidget(self._direction)

    @abstractmethod
    def is_valid(self) -> bool:
        raise NotImplementedError

    @abstractmethod
    def kv(self) -> float:
        """ [rpm/V], including efficiency """
        raise NotImplementedError

    @abstractmethod
    def time_const_up(self) -> float:
        """ [s] """
        raise NotImplementedError

    @abstractmethod
    def time_const_down(self) -> float:
        """ [s] """
        raise NotImplementedError

    @abstractmethod
    def copy_from(self, src) -> None:
        raise NotImplementedError

    @final
    def direction(self) -> str:
        """ 'cw' or 'ccw' """
        return self._direction.get().lower()


class MotorWidget_Manual(MotorWidget_Base):

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__(main, link_name)

        kv_description = "供給される電圧に対する無負荷時のモーターの回転速度．"
        self._kv = ParamGetterWidget_SpinBox(
            "Kv",
            kv_description,
            minimum=1,
            maximum=10**5,
            default=920,
            suffix=" rpm/V",
        )
        self._rows.addWidget(self._kv)

        efficiency_description = "Kvから推定される回転数に対する実際の回転数の比率．"\
            + "負荷，摩擦，電気的損失などにより実際の回転数は理論値よりも小さくなります．"
        self._efficiency = ParamGetterWidget_SpinBox(
            "Efficiency",
            efficiency_description,
            minimum=1,
            maximum=100,
            default=80,
            suffix=" %",
        )
        self._rows.addWidget(self._efficiency)

        time_const_up_description = "モータの回転数が増加する際の，指令値に対する一時遅れの時定数．"
        self._time_const_up = ParamGetterWidget_SpinBox(
            "Time Constant Up",
            time_const_up_description,
            minimum=1,
            default=10,
            suffix=" ms",
        )
        self._rows.addWidget(self._time_const_up)

        time_const_down_description = "モータの回転数が減少する際の，指令値に対する一時遅れの時定数．"
        self._time_const_down = ParamGetterWidget_SpinBox(
            "Time Constant Down",
            time_const_down_description,
            minimum=1,
            default=20,
            suffix=" ms",
        )
        self._rows.addWidget(self._time_const_down)

    def is_valid(self) -> bool:
        return True

    def kv(self) -> float:
        return self._kv.get() * (self._efficiency.get() / 100.)

    def time_const_up(self) -> float:
        return self._time_const_up.get() / 1000.

    def time_const_down(self) -> float:
        return self._time_const_down.get() / 1000.

    def copy_from(self, src: MotorWidget_Manual) -> None:
        self._direction.set(src._direction.get())
        self._kv.set(src._kv.get())
        self._efficiency.set(src._efficiency.get())
        self._time_const_up.set(src._time_const_up.get())
        self._time_const_down.set(src._time_const_down.get())


class MotorWidget_Experiment(MotorWidget_Base):

    TABLE_HEIGHT = 500
    TABLE_COL_WIDTH = 180

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__(main, link_name)

        time_const_up_description = "モータの回転数が増加する際の，指令値に対する一次遅れの時定数．"
        self._time_const_up = ParamGetterWidget_SpinBox(
            "Time Constant Up",
            time_const_up_description,
            minimum=1,
            default=10,
            suffix=" ms",
        )
        self._rows.addWidget(self._time_const_up)

        time_const_down_description = "モータの回転数が減少する際の，指令値に対する一次遅れの時定数．"
        self._time_const_down = ParamGetterWidget_SpinBox(
            "Time Constant Down",
            time_const_down_description,
            minimum=1,
            default=20,
            suffix=" ms",
        )
        self._rows.addWidget(self._time_const_down)

        data_description = "Thrust Stand実験のデータから，ESCへのPWM信号とモータの回転数の関係を推定します．"\
            + "データを直接入力するか，CSVファイルを読み込んでください．"\
            + "実験には必ず機体に搭載するバッテリーを用い，実際のプロペラを取り付けた状態で行ってください．\n"\
            + "Thrust Standの例: https://www.tytorobotics.com/pages/series-1580-1585"
        self._data = ParamGetterWidget_DoubleTable(
            "Experimental data",
            ["ESC signal", "Voltage", "Speed"],
            description_text=data_description,
        )
        self._data.set_minimum([1., 1., 1.])
        self._data.set_decimals([0, 6, 0])
        self._data.set_suffix([" us", " V", " rpm"])  # TODO: PWM型以外のESCにも対応
        self._data.set_fixed_height(self.TABLE_HEIGHT)
        self._data.set_column_width(self.TABLE_COL_WIDTH)
        self._rows.addWidget(self._data)

    def is_valid(self) -> bool:
        if self._data.count() == 0:
            q_error_named(self._main, NAME, "Experiment data is blank.")
            return False

        data = self._data.get()
        esc = self._main.settings.rotary_wings.selected.get_esc(self._link_name)
        esc_type = esc.esc_type.currentText()
        if esc_type == esc.PWM:
            lb, ub = esc.pwm.pulse_width_range.get()
            for pulse_width, _, _ in data:
                if not lb <= pulse_width <= ub:
                    q_error_named(
                        self._main, NAME, f'Pulse width out of range: {pulse_width} not in [{lb}, {ub}]',
                    )
                    return False
        elif esc_type == esc.DSHOT:
            pass  # TODO

        return True

    def kv(self) -> float:
        # TODO: 外れ値を除去
        # TODO: あまりに線形からかけ離れていたら警告を出す
        # TODO: データ数が十分かつ細かい時間間隔で取れていたらtime_constも推定できるかも

        data = self._data.get()
        num_samples = data.shape[0]
        assert num_samples > 0

        esc = self._main.settings.rotary_wings.selected.get_esc(self._link_name)
        esc_type = esc.esc_type.currentText()
        kv_sum = 0.

        if esc_type == esc.PWM:
            lb, ub = esc.pwm.pulse_width_range.get()
            for pulse_width, battery_voltage, rpm in data:
                throttle = remap(pulse_width, lb, ub, 0., 1.)
                motor_voltage = battery_voltage * throttle
                kv = rpm / motor_voltage
                kv_sum += kv
        elif esc_type == esc.DSHOT:
            raise NotImplementedError
        else:
            raise RuntimeError

        return kv_sum / num_samples

    def time_const_up(self) -> float:
        return self._time_const_up.get() / 1000.

    def time_const_down(self) -> float:
        return self._time_const_down.get() / 1000.

    def copy_from(self, src: MotorWidget_Experiment) -> None:
        self._time_const_up.set(src._time_const_up.get())
        self._time_const_down.set(src._time_const_down.get())
        self._data.set(src._data.get())
