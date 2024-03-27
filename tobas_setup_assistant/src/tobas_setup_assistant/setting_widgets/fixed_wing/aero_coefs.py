from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_std_tools_py.config_parser import ConfigParserWrapper
from tobas_rqt_tools.widgets import Widget, DoubleSpinBox
from tobas_rqt_tools.layouts import FormLayout
from tobas_rqt_tools.messages import q_info, q_error
from tobas_tools_py.constants import CONFIG_PATH

from ...parameter_getters import *
from ...common import *
from .common import STABILITY_COEF_DECIMALS


class AerodynamicsCoefficientsWidget(Widget):
    BTN_HEIGHT = 30
    BTN_WIDTH = 150
    LAST_OPENED_DIR = "aerodynamics/last_opened_dir"

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        rows = QVBoxLayout()
        self.setLayout(rows)

        label = QLabel("Aerodynamic Coefficients")
        label.setFont(QFont("Default", pointSize=TITLE_PSIZE, weight=QFont.Bold))
        label.setAlignment(Qt.AlignLeft)
        rows.addWidget(label)

        cols = QHBoxLayout()
        rows.addLayout(cols)

        self._load_button = QPushButton("Load VSPAERO Output")
        self._load_button.setFixedSize(self.BTN_WIDTH, self.BTN_HEIGHT)
        rows.addWidget(self._load_button)

        cols.addStretch()

        self._form = FormLayout()
        rows.addLayout(self._form)

        self.c_lift_0 = DoubleSpinBox()
        self.c_lift_0.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_lift_0.setSuffix(" [-]")
        self.c_lift_0.setValue(0.2127)
        self._form.addRow(QLabel("c_lift_0"), self.c_lift_0)

        self.c_lift_alpha = DoubleSpinBox()
        self.c_lift_alpha.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_lift_alpha.setSuffix(" [/rad]")
        self.c_lift_alpha.setValue(10.806)
        self._form.addRow(QLabel("c_lift_alpha"), self.c_lift_alpha)

        self.c_drag_0 = DoubleSpinBox()
        self.c_drag_0.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_drag_0.setSuffix(" [-]")
        self.c_drag_0.setValue(0.136)
        self._form.addRow(QLabel("c_drag_0"), self.c_drag_0)

        self.c_drag_alpha = DoubleSpinBox()
        self.c_drag_alpha.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_drag_alpha.setSuffix(" [/rad]")
        self.c_drag_alpha.setValue(0.6737)
        self._form.addRow(QLabel("c_drag_alpha"), self.c_drag_alpha)

        self.c_side_beta = DoubleSpinBox()
        self.c_side_beta.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_side_beta.setSuffix(" [/rad]")
        self.c_side_beta.setValue(-0.3073)
        self._form.addRow(QLabel("c_side_beta"), self.c_side_beta)

        self.c_roll_beta = DoubleSpinBox()
        self.c_roll_beta.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_roll_beta.setSuffix(" [/rad]")
        self.c_roll_beta.setValue(-0.0154)
        self._form.addRow(QLabel("c_roll_beta"), self.c_roll_beta)

        self.c_roll_p = DoubleSpinBox()
        self.c_roll_p.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_roll_p.setSuffix(" [s/rad]")
        self.c_roll_p.setValue(-0.1647)
        self._form.addRow(QLabel("c_roll_p"), self.c_roll_p)

        self.c_roll_r = DoubleSpinBox()
        self.c_roll_r.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_roll_r.setSuffix(" [s/rad]")
        self.c_roll_r.setValue(0.0117)
        self._form.addRow(QLabel("c_roll_r"), self.c_roll_r)

        self.c_pitch_0 = DoubleSpinBox()
        self.c_pitch_0.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_pitch_0.setSuffix(" [-]")
        self.c_pitch_0.setValue(0.0435)
        self._form.addRow(QLabel("c_pitch_0"), self.c_pitch_0)

        self.c_pitch_alpha = DoubleSpinBox()
        self.c_pitch_alpha.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_pitch_alpha.setSuffix(" [/rad]")
        self.c_pitch_alpha.setValue(-2.969)
        self._form.addRow(QLabel("c_pitch_alpha"), self.c_pitch_alpha)

        self.c_pitch_abs_beta = DoubleSpinBox()
        self.c_pitch_abs_beta.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_pitch_abs_beta.setSuffix(" [/rad]")
        self.c_pitch_abs_beta.setValue(0.0)
        self._form.addRow(QLabel("c_pitch_abs_beta"), self.c_pitch_abs_beta)

        self.c_pitch_alpha_rate = DoubleSpinBox()
        self.c_pitch_alpha_rate.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_pitch_alpha_rate.setSuffix(" [s/rad]")
        self.c_pitch_alpha_rate.setValue(0.0)
        self._form.addRow(QLabel("c_pitch_alpha_rate"), self.c_pitch_alpha_rate)

        self.c_pitch_q = DoubleSpinBox()
        self.c_pitch_q.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_pitch_q.setSuffix(" [s/rad]")
        self.c_pitch_q.setValue(-106.1542)
        self._form.addRow(QLabel("c_pitch_q"), self.c_pitch_q)

        self.c_yaw_beta = DoubleSpinBox()
        self.c_yaw_beta.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_yaw_beta.setSuffix(" [/rad]")
        self.c_yaw_beta.setValue(0.043)
        self._form.addRow(QLabel("c_yaw_beta"), self.c_yaw_beta)

        self.c_yaw_p = DoubleSpinBox()
        self.c_yaw_p.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_yaw_p.setSuffix(" [s/rad]")
        self.c_yaw_p.setValue(0.0)
        self._form.addRow(QLabel("c_yaw_p"), self.c_yaw_p)

        self.c_yaw_r = DoubleSpinBox()
        self.c_yaw_r.setDecimals(STABILITY_COEF_DECIMALS)
        self.c_yaw_r.setSuffix(" [s/rad]")
        self.c_yaw_r.setValue(-0.0827)
        self._form.addRow(QLabel("c_yaw_r"), self.c_yaw_r)

    def define_connections(self) -> None:
        self._load_button.clicked.connect(self._on_load_button_clicked)

    def is_valid(self) -> bool:
        return True

    def _load_params(self, file_path: str) -> None:
        with open(file_path, "r") as f:
            coefs = {
                "c_lift_0": None,
                "c_lift_alpha": None,
                "c_drag_0": None,
                "c_drag_alpha": None,
                "c_side_beta": None,
                "c_roll_beta": None,
                "c_roll_p": None,
                "c_roll_r": None,
                "c_pitch_0": None,
                "c_pitch_alpha": None,
                "c_pitch_abs_beta": None,
                "c_pitch_alpha_rate": None,
                "c_pitch_q": None,
                "c_yaw_beta": None,
                "c_yaw_p": None,
                "c_yaw_r": None,
            }

            # 行からパラメータデータを抽出
            lines = f.readlines()
            for line in lines:
                parts = line.split()
                if len(parts) != 9:
                    continue

                name, base, alpha, beta, p, q, r, mach, u = parts

                if name == "CL":
                    coefs["c_lift_0"] = float(base)
                    coefs["c_lift_alpha"] = float(alpha)
                elif name == "CD":
                    coefs["c_drag_0"] = float(base)
                    coefs["c_drag_alpha"] = float(alpha)
                elif name == "CS":
                    coefs["c_side_beta"] = float(beta)
                elif name == "CMl":
                    coefs["c_roll_beta"] = float(beta)
                    coefs["c_roll_p"] = float(p)
                    coefs["c_roll_r"] = float(r)
                elif name == "CMm":
                    coefs["c_pitch_0"] = float(base)
                    coefs["c_pitch_alpha"] = float(alpha)
                    coefs["c_pitch_abs_beta"] = float(beta)
                    coefs["c_pitch_alpha_rate"] = 0.0
                    coefs["c_pitch_q"] = float(q)
                elif name == "CMn":
                    coefs["c_yaw_beta"] = float(beta)
                    coefs["c_yaw_p"] = float(p)
                    coefs["c_yaw_r"] = float(r)

            for value in coefs.values():
                if value is None:
                    q_error(self._main, "Failed to read coefficients.")
                    return

            # フォームに反映
            self.c_lift_0.setValue(coefs["c_lift_0"])
            self.c_lift_alpha.setValue(coefs["c_lift_alpha"])
            self.c_drag_0.setValue(coefs["c_drag_0"])
            self.c_drag_alpha.setValue(coefs["c_drag_alpha"])
            self.c_side_beta.setValue(coefs["c_side_beta"])
            self.c_roll_beta.setValue(coefs["c_roll_beta"])
            self.c_roll_p.setValue(coefs["c_roll_p"])
            self.c_roll_r.setValue(coefs["c_roll_r"])
            self.c_pitch_0.setValue(coefs["c_pitch_0"])
            self.c_pitch_alpha.setValue(coefs["c_pitch_alpha"])
            self.c_pitch_abs_beta.setValue(coefs["c_pitch_abs_beta"])
            self.c_pitch_alpha_rate.setValue(coefs["c_pitch_alpha_rate"])
            self.c_pitch_q.setValue(coefs["c_pitch_q"])
            self.c_yaw_beta.setValue(coefs["c_yaw_beta"])
            self.c_yaw_p.setValue(coefs["c_yaw_p"])
            self.c_yaw_r.setValue(coefs["c_yaw_r"])

    @pyqtSlot()
    def _on_load_button_clicked(self) -> None:
        # 前回開いたパスを取得
        config = ConfigParserWrapper(CONFIG_PATH, PKG_NAME)
        config.read(CONFIG_PATH)  # 排他処理のためにこの関数内でRead & Write
        last_opened_dir = config.get(self.LAST_OPENED_DIR, fallback=osp.expanduser("~"))

        # paramsのパスを取得
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file_path, _ = QFileDialog.getOpenFileName(
            self, TITLE, last_opened_dir, "OpenVSP Stability Derivatives (*.stab)", options=options
        )

        # キャンセルの場合は何もせずに終了 (そうしないと空文字が設定されてしまう)
        if file_path == "":
            return

        # ユーザが開いたディレクトリを保存
        # closeEvent()に書くと強制終了時に呼ばれないため，ファイル読み込み時に同時に保存する
        config.set(self.LAST_OPENED_DIR, osp.dirname(file_path))
        config.write()

        # パラメータを読み込む
        self._load_params(file_path)

        q_info(self, "Coefficients are loaded successfully.")
