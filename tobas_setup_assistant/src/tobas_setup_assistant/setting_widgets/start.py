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

from dh_rqt_tools.widgets import add_spacer, add_center_button
from dh_rqt_tools.path import get_pkg_name
from dh_rqt_tools.messages import q_error
from dh_rqt_tools.roslaunch import create_launcher

from .base_setting import BaseSettingWidget
from ..common import *


class StartWidget(BaseSettingWidget):
    NAME = "Start"

    SPACE_HEIGHT = 50

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Tobas Setup Assistant"
        abst_text = (
            "Tobas Setup Assistantは，Tobasを用いてあなたのドローンのシミュレーションと制御を行うために"
            + "必要な設定ファイルを作成するのを手助けするツールです．"
            + "ここでの設定が完了すれば，すぐにあなたのドローンを飛ばすことができます．"
        )
        super().__init__(main, title_text, abst_text)

        self.setEnabled(True)  # Startだけは初めからアクティブにしておく

        self._robot_model_loader = RobotModelLoaderWidget(main)
        self._rows.addWidget(self._robot_model_loader)

        self._rows.addSpacing(self.SPACE_HEIGHT)

        self._urdf_builder_launcher = URDFBuilderLaunchder(main)
        self._rows.addWidget(self._urdf_builder_launcher)

        add_spacer(self._rows)

    @overrides
    def define_connections(self) -> None:
        self._robot_model_loader.define_connections()
        self._urdf_builder_launcher.define_connections()

    @overrides
    def is_valid(self) -> bool:
        return True


class RobotModelLoaderWidget(QWidget):
    KEY = "last_opened_dir/robot_model_loader"

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main
        self._urdf_path: str = None

        self._config = ConfigParser()

        rows = QVBoxLayout()
        self.setLayout(rows)

        label = QLabel("Description Path")
        label.setFont(QFont("Default", pointSize=LABEL_PSIZE, weight=QFont.Bold))
        label.setAlignment(Qt.AlignTop)
        rows.addWidget(label)

        instruction = Description(
            "Please set the path for the robot description and press the load button."
        )
        rows.addWidget(instruction)

        cols = QHBoxLayout()
        rows.addLayout(cols)

        self._file_text = QLineEdit("")
        self._file_text.setReadOnly(True)
        self._file_text.setFocusPolicy(Qt.NoFocus)
        cols.addWidget(self._file_text)

        self._browse_button = QPushButton("Browse")
        cols.addWidget(self._browse_button)

        self._load_button = QPushButton("Load")
        self._load_button.setEnabled(False)
        cols.addWidget(self._load_button)

    def define_connections(self) -> None:
        self._file_text.textChanged.connect(self._on_file_path_changed)
        self._browse_button.clicked.connect(self._on_browse_button_clicked)
        self._load_button.clicked.connect(self._on_load_button_clicked)
        self._main.urdf_parser.robot_model_loaded.connect(self._on_robot_model_loaded)

    @pyqtSlot()
    def _on_file_path_changed(self) -> None:
        file_path = self._file_text.text().strip()

        if not self._is_valid_path(file_path):
            self._load_button.setEnabled(False)
            return

        self._urdf_path = file_path
        self._load_button.setEnabled(True)

    @pyqtSlot()
    def _on_browse_button_clicked(self) -> None:
        # 前回開いたパスを取得
        self._config.read(CONFIG_PATH)  # 排他処理のためにこの関数内でRead & Write
        last_opened_dir = self._config.get(
            DEFAULT, self.KEY, fallback=osp.expanduser("~")
        )

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

    @pyqtSlot()
    def _on_load_button_clicked(self) -> None:
        try:
            self._launch_file()
        except Exception as e:
            q_error(self._main, f"Failed to load robot description:\n\n{e}")
            return

        self._main.signals.urdf_loaded.emit()

    def _launch_file(self) -> None:
        # description.launchで使われる環境変数を設定
        os.environ["TOBAS_SETUP_ASSISTANT_DESCRIPTION_PATH"] = self._urdf_path

        # robot_descriptionをrosparamに登録
        create_launcher(get_pkg_name(), "description.launch")

    def _is_valid_path(self, file_path: str) -> bool:
        """引数が実在するロボット記述言語かどうかを判定する．"""
        _, ext = osp.splitext(file_path)
        return ext.lower() in {".urdf", ".xacro"} and osp.isfile(file_path)

    @pyqtSlot()
    def _on_robot_model_loaded(self) -> None:
        # 無事にモデルがロードされたら，それ以降モデルの修正はできないようにする
        self._file_text.setEnabled(False)
        self._browse_button.setEnabled(False)
        self._load_button.setEnabled(False)


class URDFBuilderLaunchder(QWidget):
    BUTTON_HEIGHT = 40

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        rows = QVBoxLayout()
        self.setLayout(rows)

        instruction = Description(
            "If you do not have URDF, you can create one using URDF Builder."
        )
        rows.addWidget(instruction)

        self._open_button = add_center_button("Open URDF Builder", rows)
        self._open_button.setFixedHeight(self.BUTTON_HEIGHT)
        self._open_button.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Preferred)

    def define_connections(self) -> None:
        self._open_button.clicked.connect(self._on_open_button_clicked)
        self._main.urdf_parser.robot_model_loaded.connect(self._on_robot_model_loaded)

    @pyqtSlot()
    def _on_open_button_clicked(self) -> None:
        create_launcher("urdf_builder", "urdf_builder.launch")
        self._open_button.setEnabled(False)  # FIXME: URDF Builderを2つ立ち上げるとバグる

    @pyqtSlot()
    def _on_robot_model_loaded(self) -> None:
        self._open_button.setEnabled(False)
