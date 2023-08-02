from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

import os
import os.path as osp
import roslaunch
from configparser import ConfigParser
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import add_expanding_widget
from dh_rqt_tools.path import get_proj_path
from dh_rqt_tools.messages import q_error

from .base_setting import BaseSettingWidget
from ..constants import *


class StartWidget(BaseSettingWidget):

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Tobas Setup Assistant"
        abst_text = "Tobas Setup Assistantは，Tobasを用いてあなたのドローンのシミュレーションと制御を行うために"\
            + "必要な設定ファイルを作成するのを手助けするツールです．"\
            + "ここでの設定が完了すれば，すぐにあなたのドローンを飛ばすことができます．"
        super().__init__(main, title_text, abst_text)

        self.setEnabled(True)  # Startだけは初めからアクティブにしておく

        self.robot_model_loader = RobotModelLoaderWidget(main)
        self._rows.addWidget(self.robot_model_loader)

        add_expanding_widget(self._rows)

    def define_connections(self) -> None:
        self.robot_model_loader.define_connections()

    def is_valid(self) -> bool:
        return True


class RobotModelLoaderWidget(QWidget):

    KEY = "last_opened_dir/robot_model_loader"

    urdf_loaded = pyqtSignal()

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main
        self.description_path = None

        self._config = ConfigParser()

        description_loader_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        description_launch_path = osp.join(get_proj_path(), "launch/description.launch")
        roslaunch.configure_logging(description_loader_uuid)
        self.description_launcher = roslaunch.parent.ROSLaunchParent(
            description_loader_uuid, [description_launch_path]
        )

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        label = QLabel("Description Path")
        label.setFont(QFont("Default", pointSize=LABEL_PSIZE, weight=QFont.Bold))
        label.setAlignment(Qt.AlignTop)
        self._rows.addWidget(label)

        instruction_text = "あなたのドローンのURDFを指定し，Loadボタンを押してください．"
        instruction = QLabel(instruction_text)
        instruction.setFont(QFont("Default", pointSize=BODY_PSIZE))
        instruction.setAlignment(Qt.AlignTop)
        instruction.setWordWrap(True)
        self._rows.addWidget(instruction)

        self._cols = QHBoxLayout()
        self._rows.addLayout(self._cols)

        self.file_text = QLineEdit("")
        self.file_text.setReadOnly(True)
        self.file_text.setFocusPolicy(Qt.NoFocus)
        self._cols.addWidget(self.file_text)

        self.browse_button = QPushButton("Browse")
        self._cols.addWidget(self.browse_button)

        self.load_button = QPushButton("Load")
        self.load_button.setEnabled(False)
        self._cols.addWidget(self.load_button)

    def define_connections(self) -> None:
        self.file_text.textChanged.connect(self._on_file_path_changed)
        self.browse_button.clicked.connect(self._on_browse_button_clicked)
        self.load_button.clicked.connect(self._on_load_button_clicked)

    @pyqtSlot()
    def _on_file_path_changed(self) -> None:
        file_path = self.file_text.text().strip()

        if not self._is_valid_path(file_path):
            self.load_button.setEnabled(False)
            return

        self.description_path = file_path
        self.load_button.setEnabled(True)

    @pyqtSlot()
    def _on_browse_button_clicked(self) -> None:
        # 前回開いたパスを取得
        self._config.read(CONFIG_PATH)  # 排他処理のためにこの関数内でRead & Write
        last_opened_dir = self._config.get("DEFAULT", self.KEY, fallback=osp.expanduser("~"))

        # URDFのパスを取得
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file_path, _ = QFileDialog.getOpenFileName(
            self, TITLE, last_opened_dir, "Robot Description (*.urdf *.xacro)", options=options
        )

        # キャンセルの場合は何もせずに終了
        # でないと空文字が設定されてしまう
        if file_path == "":
            return

        # パスをテキストに設定
        self.file_text.setText(file_path)

        # ユーザが開いたディレクトリを保存
        # closeEvent()に書くと強制終了時に呼ばれないため，ファイル読み込み時に同時に保存する
        self._config["DEFAULT"][self.KEY] = osp.dirname(file_path)
        with open(CONFIG_PATH, "w") as f:
            self._config.write(f)

    @pyqtSlot()
    def _on_load_button_clicked(self) -> None:
        try:
            self._launch_file()
        except Exception as e:
            q_error(self._main, f'Failed to load robot description:\n\n{e}')
            return

        self.file_text.setEnabled(False)
        self.browse_button.setEnabled(False)
        self.load_button.setEnabled(False)

        self.urdf_loaded.emit()

    def _launch_file(self) -> None:
        # description.launchで使われる環境変数を設定
        os.environ["TOBAS_SETUP_ASSISTANT_DESCRIPTION_PATH"] = self.description_path

        # robot_descriptionをrosparamに登録
        self.description_launcher.shutdown()
        self.description_launcher.start()

    def _is_valid_path(self, file_path: str) -> bool:
        """ 引数が実在するロボット記述言語かどうかを判定する． """
        _, ext = osp.splitext(file_path)
        return ext.lower() in {'.urdf', '.xacro'} and osp.isfile(file_path)
