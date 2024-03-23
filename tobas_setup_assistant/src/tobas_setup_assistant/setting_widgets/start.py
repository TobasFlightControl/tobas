from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

import os
import os.path as osp
from overrides import overrides
from configparser import ConfigParser
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.messages import q_error
from tobas_rqt_tools.roslaunch import create_launcher

from .base_setting import BaseSettingWidget
from ..common import *


class StartWidget(BaseSettingWidget):
    NAME = "Start"

    SPACE_HEIGHT = 50

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Tobas Setup Assistant"
        abst_text = (
            "The Tobas Setup Assistant is a GUI tool designed for creating configuration files "
            "needed to operate drones with Tobas. "
            "It utilizes the URDF created in the previous steps and allows for the configuration of elements "
            "not expressed in the URDF, "
            "such as propeller aerodynamics and controller settings."
        )
        super().__init__(main, title_text, abst_text)

        self.setEnabled(True)  # Startだけは初めからアクティブにしておく

        self._robot_model_loader = RobotModelLoaderWidget(main)
        self._rows.addWidget(self._robot_model_loader)

        self._rows.addStretch()

    @overrides
    def define_connections(self) -> None:
        self._robot_model_loader.define_connections()

    @overrides
    def is_valid(self) -> bool:
        return True


class RobotModelLoaderWidget(QWidget):
    KEY = "last_opened_dir/robot_model_loader"

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._config = ConfigParser()

        rows = QVBoxLayout()
        self.setLayout(rows)

        label = QLabel("Description Path")
        label.setFont(QFont("Default", pointSize=LABEL_PSIZE, weight=QFont.Bold))
        label.setAlignment(Qt.AlignTop)
        rows.addWidget(label)

        instruction = Description("Please set the path for the robot description and press the load button.")
        rows.addWidget(instruction)

        cols = QHBoxLayout()
        rows.addLayout(cols)

        self._file_text = QLineEdit("")
        self._file_text.setReadOnly(True)
        self._file_text.setFocusPolicy(Qt.NoFocus)
        cols.addWidget(self._file_text)

        self._load_button = QPushButton("Load")
        cols.addWidget(self._load_button)

    def define_connections(self) -> None:
        self._load_button.clicked.connect(self._on_load_button_clicked)

    @pyqtSlot()
    def _on_load_button_clicked(self) -> None:
        # 前回開いたパスを取得
        self._config.read(CONFIG_PATH)  # 排他処理のためにこの関数内でRead & Write
        last_opened_dir = self._config.get(DEFAULT, self.KEY, fallback=osp.expanduser("~"))

        # URDFのパスを取得
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            TITLE,
            last_opened_dir,
            "Robot Description (*.urdf *.xacro)",
            options=options,
        )

        # キャンセルの場合は何もせずに終了 (そうしないと空文字が設定されてしまう)
        if file_path == "":
            return

        # パスをテキストに設定
        self._file_text.setText(file_path)

        # ユーザが開いたディレクトリを保存
        # closeEvent()に書くと強制終了時に呼ばれないため，ファイル読み込み時に同時に保存する
        self._config[DEFAULT][self.KEY] = osp.dirname(file_path)
        with open(CONFIG_PATH, "w") as f:
            self._config.write(f)

        # robot_descriptionをrosparamに登録
        os.environ["TOBAS_SETUP_ASSISTANT_DESCRIPTION_PATH"] = file_path
        try:
            create_launcher(PKG_NAME, "description.launch")
        except Exception as e:
            q_error(self._main, f"Failed to load robot description:\n\n{e}")
            return

        self._main.signals.urdf_loaded.emit()
