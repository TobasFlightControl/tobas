from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from overrides import overrides
from typing import List
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.messages import q_error_named
from dh_rqt_tools.widgets import ComboBox
from dh_rqt_tools.layouts import FormLayout
from kdl_sympy.frames import Vector

from tobas_msgs.msg import PositionYaw

from ...parameter_getters import *
from ...common import *
from .base import BaseController

ARDUPILOT = "ArduPilot"


class FrameClass:
    """FRAME_CLASS: https://ardupilot.org/copter/docs/parameters.html#frame-class"""

    def __init__(self, value: int, meaning: str, num_props: int) -> None:
        self.value = value
        self.meaning = meaning
        self.num_props = num_props


class FrameType:
    """FRAME_TYPE: https://ardupilot.org/copter/docs/parameters.html#frame-type"""

    def __init__(self, value: int, meaning: str) -> None:
        self.value = value
        self.meaning = meaning


class Frame:
    """Motor order diagrams: https://ardupilot.org/copter/docs/connect-escs-and-motors.html#motor-order-diagrams"""

    def __init__(
        self,
        name: str,
        frame_class: FrameClass,
        frame_type: FrameType,
        directions: List[str],
    ) -> None:
        assert len(directions) == frame_class.num_props

        self._name = name
        self._frame_class = frame_class
        self._frame_type = frame_type
        self._directions = directions

    def name(self) -> str:
        return self._name

    def direction(self, channel: int) -> str:
        """各チャンネルのプロペラの回転方向 (CW or CCW)"""
        assert (
            0 <= channel < self._frame_class.num_props
        ), f"Number of propellers: {self._frame_class.num_props}, channel: {channel}"
        return self._directions[channel]

    def directions(self) -> List[str]:
        return self._directions

    def class_id(self) -> int:
        return self._frame_class.value

    def type_id(self) -> int:
        return self._frame_type.value

    def num_props(self) -> int:
        return self._frame_class.num_props


class ChannelsWidget(QWidget):
    NO_SELECT = "Select link name"

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        label = QLabel("Channels")
        label.setFont(QFont("Default", pointSize=LABEL_PSIZE, weight=QFont.Bold))
        label.setAlignment(Qt.AlignLeft)
        self._rows.addWidget(label)

        self._form = FormLayout()
        self._rows.addLayout(self._form)

    def define_connections(self) -> None:
        self._main.signals.airframe_updated.connect(self._on_airframe_updated)

    def is_valid(self) -> bool:
        # 未選択はダメ
        for r in range(self._form.rowCount()):
            if self._link_name(r) == self.NO_SELECT:
                q_error_named(
                    self._main, ARDUPILOT, "Please select link names for each channel."
                )
                return False

        # リンク名が被ってたらダメ
        if len(set(self._link_names())) < self._form.rowCount():
            q_error_named(
                self._main,
                ARDUPILOT,
                "Link names of different channels must be different.",
            )
            return False

        return True

    def channels(self) -> List[int]:
        res = [-1] * self._form.rowCount()
        for r in range(self._form.rowCount()):
            rotor_idx = self._main.settings.propulsion_system.selected.get_index(
                self._link_name(r)
            )
            ardu_channel = r  # PIN - 1
            res[rotor_idx] = ardu_channel
        return res

    def count(self) -> int:
        """チャンネル数 (= プロペラ数)"""
        return self._form.rowCount()

    @pyqtSlot()
    def _on_airframe_updated(self) -> None:
        num_props = self._main.settings.propulsion_system.selected.count()
        if num_props != self._form.rowCount():
            self._resize(num_props)

    def _resize(self, num_props: int) -> None:
        self._form.clear()

        for i in range(num_props):
            prop_link_names = (
                self._main.settings.propulsion_system.selected.link_names()
            )
            choices = ComboBox()
            choices.addItems([self.NO_SELECT] + prop_link_names)
            choices.setCurrentText(self.NO_SELECT)
            self._form.addRow(QLabel(f"Channel {i + 1}"), choices)

    def _link_name(self, row: int):
        combo: ComboBox = self._form.get_widget(row)
        return combo.currentText()

    def _link_names(self) -> List[str]:
        return [self._link_name(row) for row in range(self._form.rowCount())]


class ArduCopter(BaseController):
    NAME = "ArduCopter (Simulation Only)"

    CONTROLLER_PKG = "tobas_mr_arducopter"
    TAKEOFF_PKG = "tobas_mr_arducopter"
    LANDING_PKG = "tobas_mr_arducopter"

    COMMAND_MSGS = [PositionYaw.__name__]

    MIN_NUM_PROPS = 2

    def __init__(self, main: SetupAssistant) -> None:
        abst_text = "ArduCopterをGazebo上でシミュレーションします．ArduPilotを用いる場合，オブザーバは使用しません．"
        super().__init__(main, abst_text)

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

        self._frames = [
            # Quadcopter
            Frame("QUAD X", quad, x, [CCW, CCW, CW, CW]),
            Frame("BETAFLIGHT X", quad, beta_x, [CW, CCW, CCW, CW]),
            Frame("BETAFLIGHT X (REVERSED)", quad, beta_x_rev, [CCW, CW, CW, CCW]),
            Frame("DJI X", quad, dji_x, [CCW, CW, CCW, CW]),
            Frame("QUAD CW X", quad, cw_x, [CCW, CW, CCW, CW]),
            Frame("QUAD +", quad, plus, [CCW, CCW, CW, CW]),
            Frame("QUAD V", quad, v, [CCW, CCW, CW, CW]),
            Frame("QUAD H", quad, h, [CW, CW, CCW, CCW]),
            Frame("QUAD-V Tail", quad, v_tail, [CCW, CCW, CW, CW]),
            Frame("QUAD-A Tail", quad, a_tail, [CCW, CCW, CW, CW]),
            Frame("Y4A", quad, y4, [CCW, CW, CCW, CW]),
            # Tricopter
            Frame("TRICOPTER", tri, none, [CCW, CCW, CCW]),
            Frame("TRICOPTER Alternative Set-up", tri, none, [CW, CCW, CCW]),
            # Bicopter
            Frame("BICOPTER", bi, i, [CCW, CW]),
            # Hexacopter
            Frame("HEXA X", hexa, x, [CW, CCW, CW, CCW, CCW, CW]),
            Frame("HEXA CW X", hexa, cw_x, [CCW, CW, CCW, CW, CCW, CW]),
            Frame("HEXA +", hexa, plus, [CW, CCW, CW, CCW, CCW, CW]),
            # Y6
            Frame("Y6A", y6, none, [CCW, CW, CCW, CW, CW, CCW]),
            Frame("Y6B", y6, y6b, [CW, CCW, CW, CCW, CW, CCW]),
            Frame("Y6F", y6, y6f, [CCW, CCW, CCW, CW, CW, CW]),
            # Octocopter
            Frame("OCTO X", octa, x, [CW, CW, CCW, CCW, CCW, CCW, CW, CW]),
            Frame("OCTO +", octa, plus, [CW, CW, CCW, CCW, CCW, CCW, CW, CW]),
            Frame("OCTO V", octa, v, [CW, CW, CCW, CCW, CCW, CCW, CW, CW]),
            Frame("OCTO H", octa, h, [CW, CW, CCW, CCW, CCW, CCW, CW, CW]),
            # OctoQuad
            Frame("OCTO QUAD X8", octa_quad, x, [CCW, CW, CCW, CW, CCW, CW, CCW, CW]),
            Frame("OCTO QUAD +", octa_quad, plus, [CCW, CW, CCW, CW, CCW, CW, CCW, CW]),
            Frame("OCTO QUAD V", octa_quad, v, [CCW, CW, CCW, CW, CCW, CW, CCW, CW]),
            Frame("OCTO QUAD H", octa_quad, h, [CW, CCW, CW, CCW, CW, CCW, CW, CCW]),
            # DodecaHexacopter
            Frame(
                "DOCECA HEXA X",
                dodeca_hexa,
                x,
                [CCW, CW, CW, CCW, CCW, CW, CW, CCW, CCW, CW, CW, CCW],
            ),
            Frame(
                "DOCECA HEXA +",
                dodeca_hexa,
                plus,
                [CCW, CW, CW, CCW, CCW, CW, CW, CCW, CCW, CW, CW, CCW],
            ),
            # Custom Frames
            # TODO
        ]

        frame_description = ""  # TODO
        self._frame = ParamGetterWidget_ComboBox(
            "Frame Type",
            frame_description,
            [frame.name() for frame in self._frames],
        )
        self._rows.addWidget(self._frame)

        self._channels = ChannelsWidget(main)
        self._rows.addWidget(self._channels)

    @overrides
    def define_connections(self) -> None:
        super().define_connections()
        self._channels.define_connections()

    @overrides
    def is_applicable(self) -> bool:
        # 固定翼は持たない
        fixed_wing = self._main.settings.fixed_wing
        if fixed_wing.has_fixed_wing.isChecked():
            return False

        # プロペラの個数条件
        prop_jnt_names = self._main.settings.propulsion_system.selected.joint_names()
        if len(prop_jnt_names) < self.MIN_NUM_PROPS:
            return False

        # Z軸正方向のプロペラのみ
        # FIXME: 特殊なドローンの場合はZ成分にシンボルが含まれる可能性がある
        for joint_name in prop_jnt_names:
            axis = self._main.urdf_parser.global_axis(joint_name)
            if not axis.is_collinear(Vector.UnitZ(), PROP_TILT_TOL):
                return False

        return True

    @overrides
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
        propulsion_system = self._main.settings.propulsion_system
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

    @overrides
    def parameter_dict(self) -> dict:
        return {
            "arducopter": {
                "frame_class": self._selected().class_id(),
                "frame_type": self._selected().type_id(),
                "channels": self._channels.channels(),
            }
        }

    def _selected(self) -> Frame:
        return self._frames[self._frame.cur_index()]
