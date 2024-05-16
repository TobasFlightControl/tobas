import os.path as osp
from typing import List
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout

from tobas_rqt_tools.widgets import Widget, ComboBox, StackedWidget
from tobas_rqt_tools.utils import qsleep
from tobas_tools_py.drone import Drone

from .apps import *
from .package_manager import PackageManagerWidget
from .shutdown_button import ShutdownButtonWidget


class GroundControlStationWidget(Widget):
    WAIT_TO_SHIFT_PAGE = 100  # [ms]

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

        self._combo_box = ComboBox()
        self._stacked_widget = StackedWidget()
        for app in self._apps:
            self._combo_box.addItem(app.NAME)
            self._stacked_widget.addWidget(app)

        # 選択リストから選択されたアプリケーションを表示
        self._combo_box.currentIndexChanged.connect(self._on_index_changed)

        self._package_manager = PackageManagerWidget(self, self._drone)
        self._shutdown_button = ShutdownButtonWidget(self, self._drone)

        # レイアウト
        rows = QVBoxLayout()
        self.setLayout(rows)
        cols = QHBoxLayout()
        rows.addLayout(cols)
        cols.addWidget(self._combo_box)
        cols.addWidget(self._package_manager)
        cols.addWidget(self._shutdown_button)
        cols.setAlignment(self._combo_box, Qt.AlignLeft)
        cols.setAlignment(self._package_manager, Qt.AlignCenter)
        cols.setAlignment(self._shutdown_button, Qt.AlignRight)
        rows.addWidget(self._stacked_widget)

        # "no attribute"エラーを防ぐため，コンストラクタの最後に再帰的にシグナルスロット接続を定義する
        self.define_connections()

    def define_connections(self) -> None:
        for app in self._apps:
            app.define_connections()

    def update_internal_data_structures(self) -> None:
        """ドローンの更新に応じて内部データを更新．"""
        for app in self._apps:
            app.update_internal_data_structures()

    def package_path(self) -> str:
        return self._package_manager.package_path()

    def package_name(self) -> str:
        return osp.basename(self.package_path())

    def package_loaded(self) -> bool:
        return self.package_path() != ""

    @pyqtSlot(int)
    def _on_index_changed(self, index: int) -> None:
        """
        QComboBoxが変更された際に，アプリケーションを遷移させる．
        QStackedWidgetのインデックスを変更するだけだと画面が壊れることがあるため，
        確実にページを遷移させるために明示的に再描画と遅延処理を入れている．
        """
        # QStackedWidgetの現在のインデックスを変更
        self._stacked_widget.setCurrentIndex(index)

        # 強制的に再描画
        self._stacked_widget.update()
        self._stacked_widget.repaint()

        # 更新時間を稼ぐために遅延させる
        qsleep(self.WAIT_TO_SHIFT_PAGE)
