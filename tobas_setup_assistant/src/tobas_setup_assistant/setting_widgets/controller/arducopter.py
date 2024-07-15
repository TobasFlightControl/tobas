from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from overrides import override
from typing import List
from dataclasses import dataclass
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout
from PyQt5.QtGui import QFont

from tobas_rqt_tools.messages import q_error_named
from tobas_rqt_tools.widgets import ComboBox
from tobas_rqt_tools.layouts import FormLayout
from tobas_kdl_sympy.frames import Vector
from tobas_tools_py.rotor_config import TurningDirection
from tobas_msgs.msg import PositionYaw

from ...common import PROP_TILT_TOL, LABEL_PSIZE, Description
from ...parameter_getters import ParamGetterWidget_ComboBox
from .base import BaseController

ARDUPILOT = "ArduPilot"


@dataclass(frozen=True)
class FrameClass:
    """FRAME_CLASS: https://ardupilot.org/copter/docs/parameters.html#frame-class"""

    value: int
    meaning: str
    num_props: int


@dataclass(frozen=True)
class FrameType:
    """FRAME_TYPE: https://ardupilot.org/copter/docs/parameters.html#frame-type"""

    value: int
    meaning: str


@dataclass(frozen=True)
class Frame:
    """Motor order diagrams: https://ardupilot.org/copter/docs/connect-escs-and-motors.html#motor-order-diagrams"""

    _name: str
    _frame_class: FrameClass
    _frame_type: FrameType
    _directions: List[str]

    def name(self) -> str:
        return self._name

    def direction(self, channel: int) -> str:
        """各チャンネルのプロペラの回転方向 (CW or CCW)"""
        assert (
            0 <= channel < self._frame_class.num_props
        ), f"Number of propellers: {self._frame_class.num_props}, channel: {channel}"
        return self._directions[channel]

    def class_id(self) -> int:
        return self._frame_class.value

    def type_id(self) -> int:
        return self._frame_type.value

    def num_props(self) -> int:
        return self._frame_class.num_props


class ChannelsWidget(QWidget):
    NO_SELECT = "Select Link Name"

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        rows = QVBoxLayout()
        self.setLayout(rows)

        label = QLabel("Channels")
        label.setFont(QFont("Default", pointSize=LABEL_PSIZE, weight=QFont.Bold))
        label.setAlignment(Qt.AlignLeft)
        rows.addWidget(label)

        description = Description(
            "On the diagram, select the link corresponding to the propeller number of the frame you have chosen."
        )
        rows.addWidget(description)

        self._form = FormLayout()
        rows.addLayout(self._form)

    def dump_settings(self) -> dict:
        res = dict()

        for i in range(self.count()):
            combo_box: ComboBox = self._form.get_widget(i)
            res[self._channel_label(i)] = combo_box.currentText()

        return res

    def load_settings(self, data: dict) -> dict:
        for i in range(self.count()):
            combo_box: ComboBox = self._form.get_widget(i)
            combo_box.setCurrentText(data[self._channel_label(i)])

    def is_valid(self) -> bool:
        # 未選択はダメ
        for r in range(self._form.rowCount()):
            if self._link_name(r) == self.NO_SELECT:
                q_error_named(self._main, ARDUPILOT, "Please select link names for each channel.")
                return False

        # リンク名が被ってたらダメ
        if len(set(self._link_names())) < self._form.rowCount():
            q_error_named(self._main, ARDUPILOT, "Link names of different channels must be different.")
            return False

        return True

    def channels(self) -> List[int]:
        res = [-1] * self._form.rowCount()
        for r in range(self._form.rowCount()):
            rotor_idx = self._main.propulsion_system.selected.get_index(self._link_name(r))
            ardu_channel = r  # PIN - 1
            res[rotor_idx] = ardu_channel
        return res

    def count(self) -> int:
        """チャンネル数 (= プロペラ数)"""
        return self._form.rowCount()

    def update_num_channels(self, num_props: int) -> None:
        # 設定ミスを防ぐため，フレームタイプが変わったら各チャンネルのリンク名をリセット
        self._form.clear()

        prop_link_names = self._main.propulsion_system.selected.link_names()
        for i in range(num_props):
            choices = ComboBox()
            choices.addItems([self.NO_SELECT] + prop_link_names)
            choices.setCurrentText(self.NO_SELECT)
            self._form.addRow(QLabel(self._channel_label(i)), choices)

    def update_link_choices(self) -> None:
        """リンク名の候補を更新．"""
        new_links = self._main.propulsion_system.selected.link_names()

        for i in range(self._form.rowCount()):
            combo: ComboBox = self._form.get_widget(i)
            cur_link = combo.currentText()

            combo.clear()
            combo.addItems([self.NO_SELECT] + new_links)

            if cur_link in new_links:
                combo.setCurrentText(cur_link)
            else:
                combo.setCurrentText(self.NO_SELECT)

    def _link_name(self, row: int):
        combo: ComboBox = self._form.get_widget(row)
        return combo.currentText()

    def _link_names(self) -> List[str]:
        return [self._link_name(row) for row in range(self._form.rowCount())]

    def _channel_label(self, idx: int) -> str:
        return f"ArduPilot CH {idx + 1}"


class ArduCopter(BaseController):
    NAME = "ArduCopter (Simulation Only)"
    CONTROLLER_PKG = "tobas_mr_arducopter"
    TAKEOFF_PKG = "tobas_mr_arducopter"
    LANDING_PKG = "tobas_dummy_pkg"  # TODO
    STABLIZE_MODE = PositionYaw.__name__
    ACROBAT_MODE = PositionYaw.__name__  # TODO
    ABST_TEXT = "To simulate ArduCopter on Gazebo, you must have ArduPilot installed."

    CHANNELS = "channels"

    MIN_NUM_PROPS = 2

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__(main)

        # Frame Classes
        quad = FrameClass(1, "Quad", 4)
        hexa = FrameClass(2, "Hexa", 6)
        octa = FrameClass(3, "Octa", 8)
        y6 = FrameClass(5, "Y6", 6)
        tri = FrameClass(7, "Tri", 3)
        octa_quad = FrameClass(4, "OctaQuad", 8)
        bi = FrameClass(10, "BiCopter", 2)
        dodeca_hexa = FrameClass(12, "DodecaHexa", 12)

        # Frame Types
        none = FrameType(-1, "None")
        plus = FrameType(0, "Plus")
        x = FrameType(1, "X")
        v = FrameType(2, "V")
        h = FrameType(3, "H")
        v_tail = FrameType(4, "V-Tail")
        a_tail = FrameType(5, "A-Tail")
        y6b = FrameType(10, "Y6B")
        y6f = FrameType(11, "Y6F")
        beta_x = FrameType(12, "BetaFlightX")
        dji_x = FrameType(13, "DJIX")
        cw_x = FrameType(14, "ClockwiseX")
        i = FrameType(15, "I")
        beta_x_rev = FrameType(18, "BetaFlightXReversed")
        y4 = FrameType(19, "Y4")

        ccw = TurningDirection.CCW.name
        cw = TurningDirection.CW.name
        self._frames = [
            # Quadcopter
            Frame("QUAD X", quad, x, [ccw, ccw, cw, cw]),
            Frame("BETAFLIGHT X", quad, beta_x, [cw, ccw, ccw, cw]),
            Frame("BETAFLIGHT X (REVERSED)", quad, beta_x_rev, [ccw, cw, cw, ccw]),
            Frame("DJI X", quad, dji_x, [ccw, cw, ccw, cw]),
            Frame("QUAD CW X", quad, cw_x, [ccw, cw, ccw, cw]),
            Frame("QUAD +", quad, plus, [ccw, ccw, cw, cw]),
            Frame("QUAD V", quad, v, [ccw, ccw, cw, cw]),
            Frame("QUAD H", quad, h, [cw, cw, ccw, ccw]),
            Frame("QUAD-V Tail", quad, v_tail, [ccw, ccw, cw, cw]),
            Frame("QUAD-A Tail", quad, a_tail, [ccw, ccw, cw, cw]),
            Frame("Y4A", quad, y4, [ccw, cw, ccw, cw]),
            # Tricopter
            Frame("TRICOPTER", tri, none, [ccw, ccw, ccw]),
            Frame("TRICOPTER Alternative Set-up", tri, none, [cw, ccw, ccw]),
            # Bicopter
            Frame("BICOPTER", bi, i, [ccw, cw]),
            # Hexacopter
            Frame("HEXA X", hexa, x, [cw, ccw, cw, ccw, ccw, cw]),
            Frame("HEXA CW X", hexa, cw_x, [ccw, cw, ccw, cw, ccw, cw]),
            Frame("HEXA +", hexa, plus, [cw, ccw, cw, ccw, ccw, cw]),
            # Y6
            Frame("Y6A", y6, none, [ccw, cw, ccw, cw, cw, ccw]),
            Frame("Y6B", y6, y6b, [cw, ccw, cw, ccw, cw, ccw]),
            Frame("Y6F", y6, y6f, [ccw, ccw, ccw, cw, cw, cw]),
            # Octocopter
            Frame("OCTO X", octa, x, [cw, cw, ccw, ccw, ccw, ccw, cw, cw]),
            Frame("OCTO +", octa, plus, [cw, cw, ccw, ccw, ccw, ccw, cw, cw]),
            Frame("OCTO V", octa, v, [cw, cw, ccw, ccw, ccw, ccw, cw, cw]),
            Frame("OCTO H", octa, h, [cw, cw, ccw, ccw, ccw, ccw, cw, cw]),
            # OctoQuad
            Frame("OCTO QUAD X8", octa_quad, x, [ccw, cw, ccw, cw, ccw, cw, ccw, cw]),
            Frame("OCTO QUAD +", octa_quad, plus, [ccw, cw, ccw, cw, ccw, cw, ccw, cw]),
            Frame("OCTO QUAD V", octa_quad, v, [ccw, cw, ccw, cw, ccw, cw, ccw, cw]),
            Frame("OCTO QUAD H", octa_quad, h, [cw, ccw, cw, ccw, cw, ccw, cw, ccw]),
            # DodecaHexacopter
            Frame("DOCECA HEXA X", dodeca_hexa, x, [ccw, cw, cw, ccw, ccw, cw, cw, ccw, ccw, cw, cw, ccw]),
            Frame("DOCECA HEXA +", dodeca_hexa, plus, [ccw, cw, cw, ccw, ccw, cw, cw, ccw, ccw, cw, cw, ccw]),
        ]

        frame_description = (
            "Please select the frame type corresponding to your aircraft from the "
            "<a href='https://ardupilot.org/copter/docs/connect-escs-and-motors.html#motor-order-diagrams'>"
        )
        self._frame = ParamGetterWidget_ComboBox(
            "Frame Type", frame_description, [frame.name() for frame in self._frames]
        )
        self._frame.index_changed.connect(self._on_frame_idx_changed)
        self._rows.addWidget(self._frame)

        self._channels = ChannelsWidget(main)
        self._rows.addWidget(self._channels)

    @override
    def update_internal_data_structures(self) -> None:
        cur_idx = self._frame.cur_index()
        self._channels.update_num_channels(self._frames[cur_idx].num_props())

    @override
    def dump_settings(self) -> dict:
        res = dict()

        res[self._frame.name()] = self._frame.get()
        res[self.CHANNELS] = self._channels.dump_settings()

        return res

    @override
    def load_settings(self, data: dict) -> None:
        self._frame.set(data[self._frame.name()])
        self._channels.load_settings(data[self.CHANNELS])

    @override
    def is_applicable(self) -> bool:
        # 固定翼は持たない
        fixed_wing = self._main.fixed_wing
        if fixed_wing.has_fixed_wing.isChecked():
            return False

        # プロペラの個数条件
        prop_jnt_names = self._main.propulsion_system.selected.joint_names()
        if len(prop_jnt_names) < self.MIN_NUM_PROPS:
            return False

        # Z軸正方向のプロペラのみ
        # FIXME: 特殊なドローンの場合はZ成分にシンボルが含まれる可能性がある
        for joint_name in prop_jnt_names:
            axis = self._main.urdf_parser.global_axis(joint_name)
            if not axis.is_collinear(Vector.UnitZ(), PROP_TILT_TOL):
                return False

        return True

    @override
    def is_valid(self) -> bool:
        selected = self._selected()

        if not self._channels.is_valid():
            return False

        # ArduPilotのフレームのプロペラ数と，推進系で指定したプロペラ数が一致していることを確認
        if self._channels.count() != selected.num_props():
            q_error_named(
                self._main,
                ARDUPILOT,
                f"Number of propellers mismatch. {selected.name()} requires {selected.num_props()} propellers.",
            )
            return False

        # 回転方向がフレームタイプと一致することを確認
        propulsion_system = self._main.propulsion_system
        channels = self._channels.channels()
        for rotor_idx in range(self._channels.count()):
            channel = channels[rotor_idx]
            rot_dir_ardu = selected.direction(channel)
            rot_dir_drone = propulsion_system.selected.direction(rotor_idx)
            if rot_dir_ardu != rot_dir_drone:
                link_name = propulsion_system.selected.link_name(rotor_idx)
                q_error_named(
                    self._main,
                    ARDUPILOT,
                    f"Rotation direction mismatch. The rotating direction of '{link_name}' is {rot_dir_drone}, "
                    + f"while the rotating direction of channel {channel + 1} is {rot_dir_ardu}.",
                )
                return False

        return True

    @override
    def static_parameters(self) -> dict:
        return {
            "frame_class": self._selected().class_id(),
            "frame_type": self._selected().type_id(),
            "channels": self._channels.channels(),
        }

    @override
    def on_opened(self) -> None:
        self._channels.update_link_choices()

    @pyqtSlot(int)
    def _on_frame_idx_changed(self, idx: int) -> None:
        self._channels.update_num_channels(self._frames[idx].num_props())

    def _selected(self) -> Frame:
        return self._frames[self._frame.cur_index()]
