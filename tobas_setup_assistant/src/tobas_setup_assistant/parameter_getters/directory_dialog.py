import os.path as osp
import rospy
from overrides import override
from typing import Optional
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QPushButton, QLineEdit, QFileDialog, QHBoxLayout

from tobas_property_tools_py.property_client import PropertyClient
from tobas_tools_py.constants import PROPERTY_SERVER_GCS

from .base import ParamGetterWidget
from ..common import TITLE, PKG_NAME


class ParamGetterWidget_DirDialog(ParamGetterWidget[str]):
    path_changed = pyqtSignal(str)

    def __init__(self, param_name: str, description_text: Optional[str] = None, default: str = "") -> None:
        super().__init__(param_name, description_text)

        # 最後に開かれたディレクトリの記録用
        self._property_client = PropertyClient(PROPERTY_SERVER_GCS, PKG_NAME)
        self._last_opened_dir_key = f'last_opened_dir/dir_dialog/{param_name.lower().replace(" ", "_")}'

        self._options = QFileDialog.Options()
        self._options |= QFileDialog.DontUseNativeDialog
        self._options |= QFileDialog.ShowDirsOnly
        self._options |= QFileDialog.DontResolveSymlinks

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

    @override
    def get(self) -> str:
        return self._path.text()

    @override
    def set(self, src: str) -> None:
        self._path.setText(src)

    @pyqtSlot(str)
    def _on_text_changed(self, text: str) -> None:
        self.path_changed.emit(text)

    @pyqtSlot()
    def _on_browse_button_clicked(self) -> None:
        res, last_opened_dir = self._property_client.get_string(self._last_opened_dir_key)
        if res < 0:
            rospy.logwarn(self._property_client.error_message())
            last_opened_dir = osp.expanduser("~")

        path = QFileDialog.getExistingDirectory(self, TITLE, last_opened_dir, self._options)
        if not path:  # Cancelの場合
            return

        self._path.setText(path)

        if self._property_client.set_string(self._last_opened_dir_key, osp.dirname(path)) < 0:
            rospy.logwarn(self._property_client.error_message())
            return
        if self._property_client.save() < 0:
            rospy.logwarn(self._property_client.error_message())
            return
