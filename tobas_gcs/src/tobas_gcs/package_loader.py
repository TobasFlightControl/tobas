from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .gcs import GroundControlStationWidget

from configparser import ConfigParser
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_rqt_tools.messages import *

from .common import *


class PackageLoaderWidget(QWidget):

    KEY = "last_opened_dir/tobas_configuration_package"

    def __init__(self, main: GroundControlStationWidget) -> None:
        super().__init__()
        self._main = main

        self._config = ConfigParser()

        cols = QHBoxLayout()
        self.setLayout(cols)

        cols.addWidget(QLabel("Tobas Package Path:"))

        self._line_edit = QLineEdit("")
        self._line_edit.setReadOnly(True)
        self._line_edit.setFocusPolicy(Qt.NoFocus)
        cols.addWidget(self._line_edit)

        self._load_button = QPushButton("Load")
        cols.addWidget(self._load_button)

    def define_connections(self) -> None:
        self._load_button.clicked.connect(self._on_load_button_clicked)

    @pyqtSlot()
    def _on_load_button_clicked(self) -> None:
        # 前回開いたパスを取得
        self._config.read(CONFIG_PATH)  # 排他処理のためにこの関数内でRead & Write
        last_opened_dir = self._config.get(
            DEFAULT, self.KEY, fallback=osp.expanduser("~")
        )

        # Tobasパッケージのパスを取得
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        options |= QFileDialog.ShowDirsOnly
        options |= QFileDialog.DontResolveSymlinks
        pkg_path = QFileDialog.getExistingDirectory(
            self, TITLE, last_opened_dir, options=options
        )

        # キャンセルの場合は何もせずに終了 (そうしないと空文字が設定されてしまう)
        if pkg_path == "":
            return

        # 有効なTobas Configuration Packageでなければ終了
        if not self._is_valid_tobas_pkg(pkg_path):
            q_error(
                self._main,
                f'"{pkg_path}" is not a Tobas configuration package or is collapsed.',
            )
            return

        # パスをテキストに設定
        self._line_edit.setText(pkg_path)

        # ユーザが開いたディレクトリを保存
        # closeEvent()に書くと強制終了時に呼ばれないため，ファイル読み込み時に同時に保存する
        self._config[DEFAULT][self.KEY] = osp.dirname(pkg_path)
        with open(CONFIG_PATH, "w") as f:
            self._config.write(f)

        # Tobasパッケージがロードされたことを通知
        self._main.signals.config_pkg_loaded.emit()

        # ロードが成功したことを示すダイアログ
        q_info(self._main, "Tobas configuration package is loaded successfully.")

    def _is_valid_tobas_pkg(self, pkg_path: str) -> bool:
        """有効なTobasパッケージかどうかを判定する．"""
        tbsf_path = osp.join(pkg_path, "config/drone.tbsf")
        if not osp.isfile(tbsf_path):
            return False

        # TODO: tbsfファイルが存在するか否か以外のチェック項目

        return True

    @pyqtSlot()
    def _on_robot_model_loaded(self) -> None:
        # 無事にモデルがロードされたら，それ以降モデルの修正はできないようにする
        self._line_edit.setEnabled(False)
        self._load_button.setEnabled(False)
        self._load_button.setEnabled(False)
