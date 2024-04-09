from overrides import override
from PyQt5.QtWidgets import QProgressDialog

from ..utils import qsleep


class ProgressDialog(QProgressDialog):
    """
    ===== QProgressDialog =====
    - show()の後に画面更新のためスリープ
    - 追加メソッド
    """

    REFLESH_SLEEP = 100  # [ms]

    @override
    def show(self) -> None:
        super().show()
        qsleep(self.REFLESH_SLEEP)

    def reflesh(self) -> None:
        # 画面更新のためにメインループを進める必要がある
        # https://stackoverflow.com/questions/47879413/pyqt-qprogressdialog-displays-as-an-empty-white-window
        qsleep(self.REFLESH_SLEEP)
