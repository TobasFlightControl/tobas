from typing import override, Optional
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QProgressDialog

from tobas_std_tools_py.math import remap

from ..utils import qsleep


class ProgressDialog(QProgressDialog):
    """
    ===== QProgressDialogとの違い =====
    - ユーザーが他のUI要素と対話できないようにする
    - タイトルを設定
    - デフォルトで最小値に設定
    - 各操作後に画面更新のためスリープ
    - 追加メソッド
    """

    REFLESH_SLEEP = 50  # [ms]

    @override
    def __init__(self, parent: Optional[QWidget] = None, title: str = "", num_steps: int = 1) -> None:
        assert num_steps > 0

        super().__init__(parent=parent)

        self.setWindowModality(Qt.WindowModality.WindowModal)  # ユーザーが他のUI要素と対話できないようにする
        self.setWindowTitle(title)

        self._num_steps = num_steps
        self._step = 0
        self.set_step(self._step)

    @override
    def show(self, reflesh: bool = True) -> None:
        super().show()
        if reflesh:
            self.reflesh()

    @override
    def setValue(self, value: int, reflesh: bool = True) -> None:
        super().setValue(value)
        if reflesh:
            self.reflesh()

    @override
    def setLabelText(self, text: str, reflesh: bool = True) -> None:
        super().setLabelText(text)
        if reflesh:
            self.reflesh()

    def set_step(self, step: int, reflesh: bool = True) -> None:
        assert 0 <= step <= self._num_steps

        self._step = step
        value = int(remap(step, 0, self._num_steps, self.minimum(), self.maximum()))
        self.setValue(value, reflesh=reflesh)

    def progress_step(self, reflesh: bool = True) -> None:
        self._step += 1
        self.set_step(self._step, reflesh=reflesh)

    def reflesh(self) -> None:
        # 画面更新のためにメインループを進める必要がある
        # https://stackoverflow.com/questions/47879413/pyqt-qprogressdialog-displays-as-an-empty-white-window
        qsleep(self.REFLESH_SLEEP)
