from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

import os
import os.path as osp
import re
import subprocess
from overrides import override
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QLabel, QPushButton, QMessageBox
from PyQt5.QtGui import QFont

from tobas_rqt_tools.path import get_catkin_ws_paths, is_in_catkin_src
from tobas_rqt_tools.utils import place_center
from tobas_rqt_tools.messages import q_error_named, QMessageLevel

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
        self.pardir = ParamGetterWidget_DirDialog("Parent Directory", pardir_description)
        self._rows.addWidget(self.pardir)

        pkg_name_description = ""
        self.pkg_name = ParamGetterWidget_LineEdit("Package Name", pkg_name_description)
        self._rows.addWidget(self.pkg_name)

        text = QLabel("The package will be generated as")
        text.setFont(QFont("Default", pointSize=BODY_PSIZE))
        text.setFixedHeight(self.TEXT_HEIGHT)
        self.setAlignment(Qt.AlignTop)
        self._rows.addWidget(text)

        self._pkg_path = PackagePath(main)
        self._rows.addWidget(self._pkg_path)

        # ボタンを中央に配置するためにLayoutとWidgetを噛ませる必要がある
        self.generate_button = QPushButton("Generate")
        self.generate_button.setFixedSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT)
        self.generate_button.setEnabled(False)
        place_center(self.generate_button, self._rows)

        self._rows.addStretch()

    @override
    def define_connections(self) -> None:
        super().define_connections()
        self._pkg_path.define_connections()
        self._pkg_path.path_changed.connect(self._on_path_changed)

    @override
    def is_valid(self) -> bool:
        pardir = self._pkg_path.pardir
        if not osp.isdir(pardir):
            q_error_named(self._main, self.NAME, f"{pardir} does not exist.")
            return False
        if not is_in_catkin_src(pardir):
            q_error_named(self._main, self.NAME, f"{pardir} is not in the src directory of a catkin workspace.")
            return False

        pkg_name = self._pkg_path.pkg_name
        if pkg_name.count("/") > 0 or pkg_name.count(" "):
            q_error_named(self._main, self.NAME, f"Invalid package name: {pkg_name}")
            return False

        pkg_path = self._pkg_path.text()
        if osp.exists(pkg_path):
            res = QMessageBox.warning(
                self._main,
                QMessageLevel.WARN.name,
                f"{pkg_path} already exists. Do you want to replace it?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if res == QMessageBox.Yes:
                subprocess.run(["rm", "-r", pkg_path])
            else:
                return False

        return True

    def pkg_path(self) -> str:
        return self._pkg_path.text()

    @pyqtSlot(str, str)
    def _on_path_changed(self, pardir: str, pkg_name: str) -> None:
        if pardir == "" or pkg_name == "":
            self.generate_button.setEnabled(False)
            return

        self.generate_button.setEnabled(True)


class PackagePath(QLabel):
    HEIGHT = 50

    path_changed = pyqtSignal(str, str)

    def __init__(self, main: SetupAssistant) -> None:
        super().__init__()
        self._main = main

        self.pardir = ""
        self.pkg_name = ""

        self.setFont(QFont("Default", pointSize=BODY_PSIZE, weight=QFont.Bold))
        self.setFixedHeight(self.HEIGHT)
        self.setAlignment(Qt.AlignTop)

        self._update()

    def define_connections(self) -> None:
        self._main.settings.ros_package.pardir.path_changed.connect(self._on_pardir_changed)
        self._main.settings.ros_package.pkg_name.text_changed.connect(self._on_pkg_name_changed)
        self._main.urdf_parser.robot_model_updated.connect(self._on_robot_model_updated)

    def _update(self) -> None:
        path = self.pardir + "/" + self.pkg_name
        path = re.sub("/*/", "/", path)  # スラッシュの重複を削除
        self.setText(path)

        self.path_changed.emit(self.pardir, self.pkg_name)

    @pyqtSlot(str)
    def _on_pardir_changed(self, pardir: str) -> None:
        self.pardir = pardir
        self._update()

    @pyqtSlot(str)
    def _on_pkg_name_changed(self, pkg_name: str) -> None:
        self.pkg_name = pkg_name
        self._update()

    @pyqtSlot()
    def _on_robot_model_updated(self) -> None:
        # デフォルトのsrcディレクトリを設定
        ws_path = self._last_accessed_ws_path()
        src_path = osp.join(ws_path, "src")
        self._main.settings.ros_package.pardir.set(src_path)

        # デフォルトのパッケージ名を設定
        pkg_name = f"tobas_{get_drone_name()}_config"
        self._main.settings.ros_package.pkg_name.set(pkg_name)

        self._update()

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
