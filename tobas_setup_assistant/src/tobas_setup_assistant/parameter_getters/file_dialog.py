import os.path as osp
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from tobas_std_tools_py.config_parser import ConfigParserWrapper
from tobas_tools_py.constants import CONFIG_PATH

from .base import ParamGetterWidget
from ..common import *


class ParamGetterWidget_FileDialog(ParamGetterWidget):
    path_changed = pyqtSignal(str)

    def __init__(
        self,
        param_name: str,
        description_text: str = None,
        default: str = "",
        initial_filter: str = "All (*)",
    ) -> None:
        super().__init__(param_name, description_text)
        self._init_filter = initial_filter

        # 最後に開かれたディレクトリの記録用
        self._config = ConfigParserWrapper(CONFIG_PATH, PKG_NAME)
        self._key = f'last_opened_dir/file_dialog/{param_name.lower().replace(" ", "_")}'

        self._options = QFileDialog.Options()
        self._options |= QFileDialog.DontUseNativeDialog

        cols = QHBoxLayout()
        self._rows.addLayout(cols)

        self._path = QLineEdit(default)
        self._path.setReadOnly(True)
        self._path.setFocusPolicy(Qt.NoFocus)
        cols.addWidget(self._path)

        self.browse_button = QPushButton("Browse")
        cols.addWidget(self.browse_button)

        self._path.textChanged.connect(self._on_text_changed)
        self.browse_button.clicked.connect(self._on_browse_button_clicked)

    def get(self) -> str:
        return self._path.text()

    def set(self, text: str) -> None:
        self._path.setText(text)

    @pyqtSlot(str)
    def _on_text_changed(self, text: str) -> None:
        self.path_changed.emit(text)

    @pyqtSlot()
    def _on_browse_button_clicked(self) -> None:
        self._config.read()
        last_opened_dir = self._config.get(self._key, fallback=osp.expanduser("~"))

        path, _ = QFileDialog.getOpenFileName(self, TITLE, last_opened_dir, self._init_filter, self._options)
        if not path:  # Cancelの場合
            return

        self._path.setText(path)

        self._config.set(self._key, osp.dirname(path))
        self._config.write()
