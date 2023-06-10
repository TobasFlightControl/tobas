from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..setup_assistant import SetupAssistant

import os.path as osp
import re
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import add_expanding_widget, add_center_button
from dh_rqt_tools.messages import q_error_named
from dh_rqt_tools.path import get_workspace_path

from .base_setting import BaseSettingWidget
from ..parameter_getters import *
from ..constants import *
from ..utils import get_drone_name


class RosPackageWidget(BaseSettingWidget):

    NAME = "ROS Package"

    TEXT_HEIGHT = 50
    BUTTON_HEIGHT = 40
    BUTTON_WIDTH = 100

    def __init__(self, main: SetupAssistant) -> None:
        title_text = "Generate ROS Package"
        abst_text = "これまでの設定を元にTobasを使用するのに必要なROSパッケージを生成します．"\
            + "パッケージのパスを指定し，Generateボタンを押してください．"
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

        self.pkg_path = PackagePath(main)
        self._rows.addWidget(self.pkg_path)

        # ボタンを中央に配置するためにLayoutとWidgetを噛ませる必要がある
        self.generate_button = add_center_button("Generate", self._rows)
        self.generate_button.setFixedSize(QSize(self.BUTTON_WIDTH, self.BUTTON_HEIGHT))
        self.generate_button.setEnabled(False)

        add_expanding_widget(self._rows)

    def define_connections(self) -> None:
        super().define_connections()
        self.pkg_path.define_connections()
        self.pkg_path.path_changed.connect(self._on_path_changed)

    def is_valid(self) -> bool:
        pardir = self.pkg_path.pardir
        if not osp.isdir(pardir):
            q_error_named(self._main, self.NAME, f'{pardir} does not exist.')
            return False

        pkg_name = self.pkg_path.pkg_name
        if pkg_name.count("/") > 0 or pkg_name.count(" "):
            q_error_named(self._main, self.NAME, f'Invalid package name: {pkg_name}')
            return False

        pkg_path = self.pkg_path.text()
        if osp.exists(pkg_path):
            q_error_named(self._main, self.NAME, f'{pkg_path} already exists.')
            return False

        return True

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
        self._main.urdf_parser.robot_model_updated.connect(self._set_defaults)

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
    def _set_defaults(self) -> None:
        par_dir = f'{get_workspace_path()}/src'
        self._main.settings.ros_package.pardir.set(par_dir)

        pkg_name = f'tobas_{get_drone_name()}_config'
        self._main.settings.ros_package.pkg_name.set(pkg_name)

        self._update()
