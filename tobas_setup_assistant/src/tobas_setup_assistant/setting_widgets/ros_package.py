from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

import os
import os.path as osp
import re
from overrides import override
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import QLabel, QPushButton
from PyQt5.QtGui import QFont

from tobas_rqt_tools.path import get_catkin_ws_paths, is_in_catkin_src
from tobas_rqt_tools.utils import place_center
from tobas_rqt_tools.messages import q_error_named, yes_or_no, QMessageLevel
from tobas_tools_py.constants import PKG_EXTENSION, CONFIG_PKG_SUFFIX, USER_PKG_SUFFIX

from ..common import BODY_PSIZE
from ..parameter_getters import ParamGetterWidget_DirDialog, ParamGetterWidget_LineEdit
from ..utils import get_drone_name
from .base_setting import BaseSettingWidget


class RosPackageWidget(BaseSettingWidget):
    NAME = "ROS Package"

    TEXT_HEIGHT = 50
    BUTTON_HEIGHT = 40
    BUTTON_WIDTH = 100

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Generate ROS Package"
        abst_text = (
            "Based on the previous settings, we will generate the necessary ROS packages for using Tobas. "
            'Please specify the path for the package and click the "Generate" button.'
        )
        super().__init__(main, title_text, abst_text)

        pardir_description = ""
        self._pardir = ParamGetterWidget_DirDialog("Parent Directory", pardir_description)
        self._pardir.path_changed.connect(self._on_path_changed)
        self._rows.addWidget(self._pardir)

        pkg_name_description = ""
        self._pkg_name = ParamGetterWidget_LineEdit("Package Name", pkg_name_description)
        self._pkg_name.text_changed.connect(self._on_path_changed)
        self._rows.addWidget(self._pkg_name)

        text = QLabel("The package will be generated as")
        text.setFont(QFont("Default", pointSize=BODY_PSIZE))
        text.setFixedHeight(self.TEXT_HEIGHT)
        self.setAlignment(Qt.AlignTop)
        self._rows.addWidget(text)

        self._pkg_path = QLabel(main)
        self._pkg_path.setFont(QFont("Default", pointSize=BODY_PSIZE, weight=QFont.Bold))
        self._pkg_path.setFixedHeight(self.TEXT_HEIGHT)
        self._pkg_path.setAlignment(Qt.AlignTop)
        self._rows.addWidget(self._pkg_path)

        # ボタンを中央に配置するためにLayoutとWidgetを噛ませる必要がある
        self._generate_button = QPushButton("Generate")
        self._generate_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self._generate_button.setEnabled(False)
        self._generate_button.clicked.connect(lambda: self._main.signals.generate_button_clicked.emit())
        place_center(self._generate_button, self._rows)

        self._rows.addStretch()

        self._main.signals.robot_model_updated.connect(self._on_robot_model_updated)

    @override
    def is_valid(self) -> bool:
        pardir = self._pardir.get()
        if not osp.isdir(pardir):
            q_error_named(self._main, self.NAME, f"{pardir} does not exist.")
            return False
        if not is_in_catkin_src(pardir):
            q_error_named(self._main, self.NAME, f"{pardir} is not in the src directory of a catkin workspace.")
            return False

        pkg_name = self._pkg_name.get()
        if pkg_name.count("/") or pkg_name.count(".") or pkg_name.count(" "):
            q_error_named(self._main, self.NAME, f"Invalid package name: {pkg_name}")
            return False

        pkg_path = self._pkg_path.text()
        if osp.exists(pkg_path):
            if not yes_or_no(self._main, f"{pkg_path} already exists. Do you want to replace it?", QMessageLevel.WARN):
                return False

        return True

    def pkg_name(self) -> str:
        return self._pkg_name.get()

    def meta_pkg_name(self) -> str:
        return self.pkg_name()

    def config_pkg_name(self) -> str:
        return self.pkg_name() + CONFIG_PKG_SUFFIX

    def user_pkg_name(self) -> str:
        return self.pkg_name() + USER_PKG_SUFFIX

    def pkg_path(self) -> str:
        return self._pkg_path.text()

    def meta_pkg_path(self) -> str:
        return osp.join(self.pkg_path(), self.meta_pkg_name())

    def config_pkg_path(self) -> str:
        return osp.join(self.pkg_path(), self.config_pkg_name())

    def user_pkg_path(self) -> str:
        return osp.join(self.pkg_path(), self.user_pkg_name())

    @pyqtSlot()
    def _on_path_changed(self) -> None:
        pardir = self._pardir.get()
        pkg_name = self._pkg_name.get()

        path = pardir + "/" + pkg_name + PKG_EXTENSION
        path = re.sub("/*/", "/", path)  # スラッシュの重複を削除
        self._pkg_path.setText(path)

        self._generate_button.setEnabled(pardir != "" and pkg_name != "")

    @pyqtSlot()
    def _on_robot_model_updated(self) -> None:
        # デフォルトのsrcディレクトリを設定
        ws_path = self._last_accessed_ws_path()
        src_path = osp.join(ws_path, "src")
        self._pardir.set(src_path)

        # デフォルトのパッケージ名を設定
        pkg_name = f"tobas_{get_drone_name()}"
        self._pkg_name.set(pkg_name)

    def _last_accessed_ws_path(self) -> str:
        catkin_ws_paths = get_catkin_ws_paths()

        # もしcatkin_wsが存在しなければ作る
        if len(catkin_ws_paths) == 0:
            ws_path = osp.expanduser("~/catkin_ws/")
            src_path = osp.join(ws_path, "src")
            os.makedirs(src_path, exist_ok=True)
            os.chdir(ws_path)
            if os.system("catkin init") != 0:
                raise RuntimeError("Failed to create catkin workspace.")
            return ws_path

        cnd_ws = None
        cnd_time = 0
        for ws in catkin_ws_paths:
            last_accessed_time = osp.getatime(ws)
            if last_accessed_time > cnd_time:
                cnd_ws = ws
                cnd_time = last_accessed_time

        return cnd_ws
