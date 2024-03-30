from overrides import override
from PyQt5.QtWidgets import QComboBox
from PyQt5.QtGui import QWheelEvent


class ComboBox(QComboBox):
    """
    ===== QComboBoxとの違い =====
    - マウスホイールイベントを無効化
    - 追加メソッド
    """

    @override
    def wheelEvent(self, e: QWheelEvent) -> None:
        e.ignore()

    def contains(self, text: str) -> bool:
        for idx in range(self.count()):
            if self.itemText(idx) == text:
                return True
        return False

    def remove_text(self, text: str) -> None:
        idx = self.findText(text)
        self.removeItem(idx)
