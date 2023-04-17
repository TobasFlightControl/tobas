from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

from abc import abstractmethod
from typing import List, final
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import ComboBox
from dh_rqt_tools.messages import *

from .base_setting import BaseSettingWidget
from ..parameter_getters import *
from ..constants import *
from ..utils import add_expanding_widget, add_center_button


class RotaryWingsWidget(BaseSettingWidget):

    LABEL_PSIZE = 12

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Define Rotary Wings"
        abst_text = "TODO: abstruct"
        super().__init__(main, title_text, abst_text)

        links_label = QLabel("Available Links")
        links_label.setFont(QFont("Default", pointSize=self.LABEL_PSIZE, weight=QFont.Bold))
        links_label.setAlignment(Qt.AlignLeft)
        self._rows.addWidget(links_label)

        self.available = AvailableLinksWidget(self._main)
        self._rows.addWidget(self.available)

        self.selected = SelectedLinksWidget(self._main)
        self._rows.addWidget(self.selected)

        add_expanding_widget(self._rows)

    def define_connections(self) -> None:
        super().define_connections()
        self.available.define_connections()


class AvailableLinksWidget(QListWidget):

    ITEM_HEIGHT = 40

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

    def define_connections(self) -> None:
        self._main.urdf_parser.robot_model_updated.connect(self._add_available_links)

    def add(self, link_name: str) -> None:
        assert self._main.urdf_parser.link_exists(link_name), f'Unknown link: {link_name}'
        assert not self._link_exists_in_list(link_name), f'Duplicated: {link_name}'

        item = QListWidgetItem()
        item.setSizeHint(QSize(0, self.ITEM_HEIGHT))  # 横幅が小さすぎる場合は自動で引き伸ばされる
        self.addItem(item)
        self.setItemWidget(item, AvailableLinkItemWidget(self._main, link_name))

    @pyqtSlot(str)
    def remove(self, link_name: str) -> None:
        for row in range(self.count()):
            item = self.item(row)
            link_widget: AvailableLinkItemWidget = self.itemWidget(item)
            if link_widget.link_name() == link_name:
                self.takeItem(row)
                return
        else:
            raise RuntimeError(f'Link name not found: {link_name}')

    @pyqtSlot()
    def _add_available_links(self) -> None:
        """ rootから複数のfixedと1つのcontinuousで繋がったリンクのみプロペラ候補とする． """
        root_link = self._main.urdf_parser.get_root()
        links = self._main.urdf_parser.get_links()
        fixed_link_names = self._main.urdf_parser.get_fixed_link_names()

        for link in links:
            if link.name == root_link.name:
                continue

            joint = self._main.urdf_parser.get_joint(link.name)
            parent = self._main.urdf_parser.get_parent(link.name)
            if joint.type == "continuous" and parent.name in fixed_link_names:
                self.add(link.name)

        self.sortItems()

    def _link_exists_in_list(self, link_name: str) -> bool:
        items = self.findItems(link_name, Qt.MatchExactly)
        return len(items) > 0


class AvailableLinkItemWidget(QListWidget):

    BUTTON_HEIGHT = 20
    BUTTON_WIDTH = 60

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__()
        self._main = main

        self._cols = QHBoxLayout()
        self.setLayout(self._cols)

        self._link_label = QLabel(link_name)
        self._link_label.setFont(QFont("Default", pointSize=BODY_PSIZE))
        self._link_label.setAlignment(Qt.AlignLeft)
        self._cols.addWidget(self._link_label)

        self._add_button = QPushButton("Add")
        self._add_button.setFixedSize(QSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT))
        self._cols.addWidget(self._add_button)

        self._define_connections()

    def link_name(self) -> str:
        return self._link_label.text()

    def _define_connections(self) -> None:
        self._add_button.clicked.connect(self._on_add_button_clicked)

    @pyqtSlot()
    def _on_add_button_clicked(self) -> None:
        self._main.settings.rotary_wings.selected.add(self.link_name())
        self._main.settings.rotary_wings.available.remove(self.link_name())


class SelectedLinksWidget(QTabWidget):

    TAB_HEIGHT = 40
    TAB_WIDTH = 100

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self.settings: List[SelectedLinkTabWidget] = []

        self.setStyleSheet(
            f'QTabBar::tab {{ height: {self.TAB_HEIGHT}px; width: {self.TAB_WIDTH}px; }}'
        )

    def add(self, link_name: str) -> None:
        setting = SelectedLinkTabWidget(self._main, link_name)
        self.settings.append(setting)
        self.addTab(setting, link_name)

    def get_index(self, link_name) -> int:
        """ タブのインデックスを返す． """
        for idx, setting in enumerate(self.settings):
            if setting.link_name() == link_name:
                return idx
        else:
            raise RuntimeError(f'Link name not found: {link_name}')

    def link_names(self) -> List[str]:
        """ 選択テーブル内のリンクの名前のリストを返す． """
        return [setting.link_name() for setting in self.settings]

    def joint_names(self) -> List[str]:
        """ 選択テーブル内のジョイントの名前のリストを返す． """
        return [setting.joint_name() for setting in self.settings]

    def count(self) -> int:
        """ 登録された回転翼の個数． """
        return len(self.settings)


class SelectedLinkTabWidget(QWidget):

    CP_BUTTON_HEIGHT = 40
    CP_BUTTON_WIDTH = 150
    RM_BUTTON_HEIGHT = 40
    RM_BUTTON_WIDTH = 100

    def __init__(self, main: SetupAssistant, link_name: str) -> None:
        super().__init__()
        self._main = main
        self._link_name = link_name

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        self.copy_button = add_center_button("Copy from left tab", self._rows)
        self.copy_button.setFixedSize(QSize(self.CP_BUTTON_WIDTH, self.CP_BUTTON_HEIGHT))

        self.esc = EscWidget()
        self._rows.addWidget(self.esc)

        self.motor = MotorWidget()
        self._rows.addWidget(self.motor)

        self.aerodynamics = AerodynamicsWidget()
        self._rows.addWidget(self.aerodynamics)

        self.remove_button = add_center_button("Remove", self._rows)
        self.remove_button.setFixedSize(QSize(self.RM_BUTTON_WIDTH, self.RM_BUTTON_HEIGHT))
        # self.remove_button.setStyleSheet("background-color: rgb(255,0,0);")

        add_expanding_widget(self._rows)
        self._define_connections()

    def link_name(self) -> str:
        return self._link_name

    def joint_name(self) -> str:
        return self._main.urdf_parser.get_joint(self._link_name).name

    def _define_connections(self) -> None:
        self.copy_button.clicked.connect(self._copy_from_left_tab)
        self.remove_button.clicked.connect(self._on_remove_button_clicked)

    @pyqtSlot()
    def _copy_from_left_tab(self) -> None:
        selected = self._main.settings.rotary_wings.selected
        self_idx = selected.get_index(self._link_name)

        if self_idx == 0:
            q_info(self._main, "No left tab")
            return

        left = selected.settings[self_idx - 1]
        self.esc.copy_from(left.esc)
        self.motor.copy_from(left.motor)
        self.aerodynamics.copy_from(left.aerodynamics)

    @pyqtSlot()
    def _on_remove_button_clicked(self) -> None:
        raise NotImplementedError


class EscWidget(QWidget):

    NO_SELECT = "Select ESC type"
    PWM = "PWM"
    DSHOT = "D-SHOT"

    def __init__(self) -> None:
        super().__init__()

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        title = QLabel("ESC Settings")
        title.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Bold))
        title.setAlignment(Qt.AlignTop)
        self._rows.addWidget(title)

        self.esc_type = ComboBox()
        self.esc_type.addItems([self.NO_SELECT, self.PWM])
        self.esc_type.setCurrentText(self.NO_SELECT)
        self._rows.addWidget(self.esc_type)

        self.pwm = EscWidget_PWM()
        self._rows.addWidget(self.pwm)

        self.dshot = EscWidget_DSHOT()
        self._rows.addWidget(self.dshot)

        self._update_visibility()
        self._define_connections()

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

    def __init__(self) -> None:
        super().__init__()

    @abstractmethod
    def copy_from(self, src) -> None:
        pass


class EscWidget_PWM(QWidget):

    def __init__(self) -> None:
        super().__init__()

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        freq_description = "TODO: instruction"
        self.freq = ParamGetterWidget_SpinBox(
            "Frequency",
            freq_description,
            minimum=1,
            default=50,
            suffix=" Hz",
        )
        self._rows.addWidget(self.freq)

        pulse_width_range_description = "TODO: instruction"
        self.pulse_width_range = ParamGetterWidget_IntRange(
            "Pulse Width Range",
            pulse_width_range_description,
            minimum=1,
            default=(1000, 2000),
            suffix=" us",
        )
        self._rows.addWidget(self.pulse_width_range)

    def copy_from(self, src: EscWidget_PWM) -> None:
        self.freq.set(src.freq.get())
        self.pulse_width_range.set(*src.pulse_width_range.get())


class EscWidget_DSHOT(QWidget):

    def __init__(self) -> None:
        super().__init__()

        # TODO

    def copy_from(self, src: EscWidget_DSHOT) -> None:
        pass  # TODO


class MotorWidget(QWidget):

    NO_SELECT = "Select setting method"
    MANUAL = "Set manually"
    EXPERIMENT = "Set from experimental data"

    def __init__(self) -> None:
        super().__init__()

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

        self.manual = MotorWidget_Manual()
        self._rows.addWidget(self.manual)

        self.experiment = MotorWidget_Experiment()
        self._rows.addWidget(self.experiment)

        self._update_visibility()
        self._define_connections()

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
            raise RuntimeError

    @pyqtSlot(str)
    def _on_type_changed(self, setting_method: str) -> None:
        self._update_visibility()


class MotorWidget_Base(QWidget):  # ABCを継承するとバグる

    def __init__(self) -> None:
        super().__init__()

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        direction_description = "TODO: instruction"
        self._direction = ParamGetterWidget_ComboBox(
            "Rotating Direction",
            direction_description,
            ["CW", "CCW"],
        )
        self._rows.addWidget(self._direction)

    @final
    def direction(self) -> str:
        """ 'cw' or 'ccw' """
        return self._direction.get().lower()

    @abstractmethod
    def kv(self) -> float:
        """ [rpm/V], including efficiency """
        pass

    @abstractmethod
    def time_const_up(self) -> float:
        """ [s] """
        pass

    @abstractmethod
    def time_const_down(self) -> float:
        """ [s] """
        pass

    @abstractmethod
    def copy_from(self, src) -> None:
        pass


class MotorWidget_Manual(MotorWidget_Base):

    def __init__(self) -> None:
        super().__init__()

        kv_description = "TODO: instruction"
        self._kv = ParamGetterWidget_SpinBox(
            "Kv",
            kv_description,
            minimum=1,
            maximum=10**5,
            default=920,
            suffix=" rpm/V",
        )
        self._rows.addWidget(self._kv)

        efficiency_description = "TODO: instruction"
        self._efficiency = ParamGetterWidget_SpinBox(
            "Efficiency",
            efficiency_description,
            minimum=1,
            maximum=100,
            default=80,
            suffix=" %",
        )
        self._rows.addWidget(self._efficiency)

        time_const_up_description = "TODO: instruction"
        self._time_const_up = ParamGetterWidget_SpinBox(
            "Time Constant Up",
            time_const_up_description,
            minimum=1,
            default=10,
            suffix=" ms",
        )
        self._rows.addWidget(self._time_const_up)

        time_const_down_description = "TODO: instruction"
        self._time_const_down = ParamGetterWidget_SpinBox(
            "Time Constant Down",
            time_const_down_description,
            minimum=1,
            default=20,
            suffix=" ms",
        )
        self._rows.addWidget(self._time_const_down)

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

    def __init__(self) -> None:
        super().__init__()

        # TODO

    def copy_from(self, src: MotorWidget_Experiment) -> None:
        pass  # TODO


class AerodynamicsWidget(QWidget):

    NO_SELECT = "Select setting method"
    MANUAL = "Set manually"
    BLADE_THEORY = "Set from blade property"
    EXPERIMENT = "Set from experimental data"

    def __init__(self) -> None:
        super().__init__()

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        title = QLabel("Aerodynamics")
        title.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Bold))
        title.setAlignment(Qt.AlignTop)
        self._rows.addWidget(title)

        self.setting_method = ComboBox()
        self.setting_method.addItems(
            [self.NO_SELECT, self.MANUAL, self.BLADE_THEORY, self.EXPERIMENT]
        )
        self.setting_method.setCurrentText(self.NO_SELECT)
        self._rows.addWidget(self.setting_method)

        self.manual = AerodynamicsWidget_Manual()
        self._rows.addWidget(self.manual)

        self.blade_theory = AerodynamicsWidget_BladeTheory()
        self._rows.addWidget(self.blade_theory)

        self.experiment = AerodynamicsWidget_Experiment()
        self._rows.addWidget(self.experiment)

        self._update_visibility()
        self._define_connections()

    def motor_const(self) -> float:
        """ [kg*m/s^2] """
        return self._selected_setting_widget().motor_const()

    def moment_const(self) -> float:
        """ [m] """
        return self._selected_setting_widget().moment_const()

    def rotor_drag_coef(self) -> float:
        """ [Ns^2/m^2] """
        return self._selected_setting_widget().rotor_drag_coef()

    def copy_from(self, src) -> None:
        pass

    def copy_from(self, src: AerodynamicsWidget) -> None:
        self.setting_method.setCurrentText(src.setting_method.currentText())
        self.manual.copy_from(src.manual)
        self.blade_theory.copy_from(src.blade_theory)
        self.experiment.copy_from(src.experiment)

        self._update_visibility()

    def _define_connections(self) -> None:
        self.setting_method.currentTextChanged.connect(self._on_type_changed)

    def _update_visibility(self) -> None:
        setting_method = self.setting_method.currentText()

        if setting_method == self.NO_SELECT:
            self.manual.setVisible(False)
            self.blade_theory.setVisible(False)
            self.experiment.setVisible(False)
        elif setting_method == self.MANUAL:
            self.manual.setVisible(True)
            self.blade_theory.setVisible(False)
            self.experiment.setVisible(False)
        elif setting_method == self.BLADE_THEORY:
            self.manual.setVisible(False)
            self.blade_theory.setVisible(True)
            self.experiment.setVisible(False)
        elif setting_method == self.EXPERIMENT:
            self.manual.setVisible(False)
            self.blade_theory.setVisible(False)
            self.experiment.setVisible(True)
        else:
            raise RuntimeError(f'Unknown setting method: {setting_method}')

    def _selected_setting_widget(self) -> AerodynamicsWidget_Base:
        setting_method = self.setting_method.currentText()

        if setting_method == self.MANUAL:
            return self.manual
        elif setting_method == self.BLADE_THEORY:
            return self.blade_theory
        elif setting_method == self.EXPERIMENT:
            return self.experiment
        else:
            raise RuntimeError

    @pyqtSlot(str)
    def _on_type_changed(self, setting_method: str) -> None:
        self._update_visibility()


class AerodynamicsWidget_Base(QWidget):

    def __init__(self) -> None:
        super().__init__()

    @abstractmethod
    def motor_const(self) -> float:
        """ [kg*m/s^2] """
        pass

    @abstractmethod
    def moment_const(self) -> float:
        """ [m] """
        pass

    @abstractmethod
    def rotor_drag_coef(self) -> float:
        """ [Ns^2/m^2] """
        pass

    @abstractmethod
    def copy_from(self, src) -> None:
        pass


class AerodynamicsWidget_Manual(AerodynamicsWidget_Base):

    def __init__(self) -> None:
        super().__init__()

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        motor_const_description = "TODO: instruction"
        self._motor_const = ParamGetterWidget_DoubleSpinBox(
            "Motor Constant",
            motor_const_description,
            decimals=12,
            minimum=0.,
            default=8.54858e-6,
            suffix=" kg*m/s^2",
        )
        self._rows.addWidget(self._motor_const)

        moment_const_description = "TODO: instruction"
        self._moment_const = ParamGetterWidget_DoubleSpinBox(
            "Moment Constant",
            moment_const_description,
            decimals=6,
            minimum=0.,
            default=0.016,
            suffix=" m",
        )
        self._rows.addWidget(self._moment_const)

        rotor_drag_coef_description = "TODO: instruction"
        self._rotor_drag_coef = ParamGetterWidget_DoubleSpinBox(
            "Rotor Drag Coefficient",
            rotor_drag_coef_description,
            decimals=9,
            minimum=0.,
            default=8.06428e-5,
            suffix=" Ns^2/m^2",
        )
        self._rows.addWidget(self._rotor_drag_coef)

    def motor_const(self) -> float:
        return self._motor_const.get()

    def moment_const(self) -> float:
        """ [m] """
        return self._moment_const.get()

    def rotor_drag_coef(self) -> float:
        """ [Ns^2/m^2] """
        return self._rotor_drag_coef.get()

    def copy_from(self, src: AerodynamicsWidget_Manual) -> None:
        self._motor_const.set(src._motor_const.get())
        self._moment_const.set(src._moment_const.get())
        self._rotor_drag_coef.set(src._rotor_drag_coef.get())


class AerodynamicsWidget_BladeTheory(AerodynamicsWidget_Base):

    def __init__(self) -> None:
        super().__init__()

        # TODO

    def copy_from(self, src: AerodynamicsWidget_BladeTheory) -> None:
        pass  # TODO


class AerodynamicsWidget_Experiment(AerodynamicsWidget_Base):

    def __init__(self) -> None:
        super().__init__()

        # TODO

    def copy_from(self, src: AerodynamicsWidget_Experiment) -> None:
        return  # TODO
