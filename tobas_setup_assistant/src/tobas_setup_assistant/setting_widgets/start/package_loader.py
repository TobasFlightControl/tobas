from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

import os
import os.path as osp
import yaml
import rospy
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import QLabel, QLineEdit, QPushButton, QFileDialog, QVBoxLayout, QHBoxLayout
from PyQt5.QtGui import QFont

from tobas_property_tools_py.property_client import PropertyClient
from tobas_rqt_tools.widgets import Widget
from tobas_rqt_tools.messages import q_info, q_error
from tobas_rqt_tools.roslaunch import launch
from tobas_tools_py.constants import GCS_NAMESPACE, PKG_EXTENSION
from tobas_tools_py.package import get_urdf_path, get_settings_path
from tobas_tools_py.command import source_tobas_package

from ...common import TITLE, PKG_NAME, LABEL_PSIZE, Description


class PackageLoaderWidget(Widget):
    LAST_OPENED_DIR_KEY = "last_opened_dir/package_loader"

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._property_client = PropertyClient(GCS_NAMESPACE, PKG_NAME)

        rows = QVBoxLayout()
        self.setLayout(rows)

        label = QLabel("Tobas Configuration Package Path")
        label.setFont(QFont("Default", pointSize=LABEL_PSIZE, weight=QFont.Bold))
        label.setAlignment(Qt.AlignTop)
        rows.addWidget(label)

        instruction = Description("Please set the path for the Tobas configuration package and press the load button.")
        rows.addWidget(instruction)

        cols = QHBoxLayout()
        rows.addLayout(cols)

        self._file_text = QLineEdit()
        self._file_text.setReadOnly(True)
        self._file_text.setFocusPolicy(Qt.NoFocus)
        cols.addWidget(self._file_text)

        self._load_button = QPushButton("Load")
        self._load_button.clicked.connect(self._on_load_button_clicked)
        cols.addWidget(self._load_button)

        rows.addStretch()

    @pyqtSlot()
    def _on_load_button_clicked(self) -> None:
        # 前回開いたパスを取得
        res, last_opened_dir = self._property_client.get_string(self.LAST_OPENED_DIR_KEY)
        if res < 0:
            rospy.logwarn(self._property_client.error_message())
            last_opened_dir = osp.expanduser("~")

        # Tobasパッケージのパスを取得
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        options |= QFileDialog.ShowDirsOnly
        options |= QFileDialog.DontResolveSymlinks
        tbs_path = QFileDialog.getExistingDirectory(self, TITLE, last_opened_dir, options=options)
        assert not tbs_path.endswith("/")  # NOTE: スラッシュで終わる場合はosp.dirname, osp.basename等の挙動が変わる

        # キャンセルの場合は何もせずに終了 (そうしないと空文字が設定されてしまう)
        if tbs_path == "":
            return

        # 拡張子をチェック
        if not tbs_path.endswith(PKG_EXTENSION):
            q_error(self._main, f'"{tbs_path}" is not a Tobas configuration package (*{PKG_EXTENSION}).')
            return

        # パスをテキストに設定
        self._file_text.setText(tbs_path)

        # ユーザが開いたディレクトリを保存
        if self._property_client.set_string(self.LAST_OPENED_DIR_KEY, osp.dirname(tbs_path)) < 0:
            q_error(self._property_client.error_message())
        if self._property_client.save() < 0:
            q_error(self._property_client.error_message())

        # Tobasパッケージのパスを追加する
        source_tobas_package(tbs_path)

        # robot_descriptionをrosparamに登録
        os.environ["TOBAS_SETUP_ASSISTANT_DESCRIPTION_PATH"] = f"{get_urdf_path(tbs_path)} DEBUG:=false"
        process = launch(PKG_NAME, "description.launch")
        _, stderr = process.communicate()
        if process.returncode != 0:
            error_msg = stderr.decode() if isinstance(stderr, bytes) else "Unknown Error"
            q_error(self._main, f"Failed to load robot description:\n\n{error_msg}")
            return

        # URDFを解析
        if not self._main.urdf_parser.load_from_param():
            return

        # URDFを各ウィジェットに反映
        self._main.update_internal_data_structures()

        # ユーザ設定を読み込む
        settings_path = get_settings_path(tbs_path)
        with open(settings_path, "r") as f:
            settings = yaml.safe_load(f)
        if not self._main.load_settings(settings):
            return

        q_info(self._main, "Tobas configuration package is loaded successfully.")
