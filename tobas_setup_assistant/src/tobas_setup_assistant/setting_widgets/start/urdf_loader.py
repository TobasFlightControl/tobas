from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ...setup_assistant import SetupAssistant

import os
import os.path as osp
import rclpy
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import (
    QLabel,
    QLineEdit,
    QPushButton,
    QFileDialog,
    QVBoxLayout,
    QHBoxLayout,
)
from PyQt5.QtGui import QFont

from tobas_property_tools_py.property_client import PropertyClient
from tobas_rqt_tools.widgets import Widget
from tobas_rqt_tools.messages import q_info, q_error
from tobas_rqt_tools.roslaunch import launch
from tobas_tools_py.constants import PROPERTY_SERVER_GCS

from ...common import TITLE, PKG_NAME, LABEL_PSIZE, Description


class URDFLoaderWidget(Widget):
    LAST_OPENED_DIR_KEY = "last_opened_dir/urdf_loader"

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self._property_client = PropertyClient(PROPERTY_SERVER_GCS, PKG_NAME)

        rows = QVBoxLayout()
        self.setLayout(rows)

        label = QLabel("Description Path")
        label.setFont(QFont("Default", pointSize=LABEL_PSIZE, weight=QFont.Weight.Bold))
        label.setAlignment(Qt.AlignmentFlag.AlignTop)
        rows.addWidget(label)

        instruction = Description("Please set the path for the robot description and press the load button.")
        rows.addWidget(instruction)

        cols = QHBoxLayout()
        rows.addLayout(cols)

        self._file_text = QLineEdit()
        self._file_text.setReadOnly(True)
        self._file_text.setFocusPolicy(Qt.FocusPolicy.NoFocus)
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
            rclpy.logwarn(self._property_client.error_message())
            last_opened_dir = osp.expanduser("~")

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
        if self._property_client.set_string(self.LAST_OPENED_DIR_KEY, osp.dirname(file_path)) < 0:
            self.get_logger().error(self._property_client.error_message())
        if self._property_client.save() < 0:
            self.get_logger().error(self._property_client.error_message())

        # robot_descriptionをrosparamに登録
        os.environ["TOBAS_SETUP_ASSISTANT_DESCRIPTION_PATH"] = file_path
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

        q_info(
            self._main,
            "URDF is loaded successfully. Configure the settings for each tab.",
        )
