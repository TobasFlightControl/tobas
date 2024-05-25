import os.path as osp
from typing import List
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout

from tobas_std_tools_py.path import get_basename_without_extension
from tobas_rqt_tools.widgets import Widget, ComboBox, StackedWidget
from tobas_tools_py.constants import CONFIG_PKG_SUFFIX, USER_PKG_SUFFIX
from tobas_tools_py.drone import Drone

from .apps import *
from .package_manager import PackageManagerWidget
from .shutdown_button import ShutdownButtonWidget


class GroundControlStationWidget(Widget):
    def __init__(self) -> None:
        super().__init__()

        self._drone = Drone()

        self._apps: List[BaseAppWidget] = [
            StartWidget(self, self._drone),
            UrdfBuilderWidget(self, self._drone),
            SetupAssistantWidget(self, self._drone),
            HardwareSetupWidget(self, self._drone),
            SimulationWidget(self, self._drone),
            MissionPlannerWidget(self, self._drone),
            ControlSystemWidget(self, self._drone),
            ParameterTuningWidget(self, self._drone),
            ConsoleWidget(self, self._drone),
        ]

        combo_box = ComboBox()
        stacked_widget = StackedWidget()
        for app in self._apps:
            combo_box.addItem(app.NAME)
            stacked_widget.addWidget(app)

        # 選択リストから選択されたアプリケーションを表示
        combo_box.currentIndexChanged.connect(stacked_widget.setCurrentIndex)

        self._package_manager = PackageManagerWidget(self, self._drone)
        self._shutdown_button = ShutdownButtonWidget(self, self._drone)

        # レイアウト
        rows = QVBoxLayout()
        self.setLayout(rows)
        cols = QHBoxLayout()
        rows.addLayout(cols)
        cols.addWidget(combo_box)
        cols.addWidget(self._package_manager)
        cols.addWidget(self._shutdown_button)
        cols.setAlignment(combo_box, Qt.AlignLeft)
        cols.setAlignment(self._package_manager, Qt.AlignCenter)
        cols.setAlignment(self._shutdown_button, Qt.AlignRight)
        rows.addWidget(stacked_widget)

    def update_internal_data_structures(self) -> None:
        """ドローンの更新に応じて内部データを更新．"""
        for app in self._apps:
            app.update_internal_data_structures()

    def pkg_name(self) -> str:
        return get_basename_without_extension(self.pkg_path())

    def meta_pkg_name(self) -> str:
        return self.pkg_name()

    def config_pkg_name(self) -> str:
        return self.pkg_name() + CONFIG_PKG_SUFFIX

    def user_pkg_name(self) -> str:
        return self.pkg_name() + USER_PKG_SUFFIX

    def pkg_path(self) -> str:
        return self._package_manager.package_path()

    def meta_pkg_path(self) -> str:
        return osp.join(self.pkg_path(), self.meta_pkg_name())

    def config_pkg_path(self) -> str:
        return osp.join(self.pkg_path(), self.config_pkg_name())

    def user_pkg_path(self) -> str:
        return osp.join(self.pkg_path(), self.user_pkg_name())

    def pkg_loaded(self) -> bool:
        return self.pkg_path() != ""
