from overrides import override
from PyQt5.QtWidgets import QComboBox
from PyQt5.QtGui import QWheelEvent


class ComboBox(QComboBox):
    """
    ===== QComboBoxとの違い =====
    - マウスホイールイベントを無効化
    - setCurrentIndexで範囲チェック
    - setCurrentTextで存在しない選択肢を指定するとエラー
    - 追加メソッド
    """

    @override
    def wheelEvent(self, e: QWheelEvent) -> None:
        e.ignore()

    @override
    def setCurrentIndex(self, index: int) -> None:
        if index < 0 or self.count() <= index:
            raise RuntimeError(f"Index {index} is out of range.")
        super().setCurrentIndex(index)

    @override
    def setCurrentText(self, text: str) -> None:
        if not self.contains(text):
            raise RuntimeError(f'The choices does not contain "{text}".')
        super().setCurrentText(text)

    def contains(self, text: str) -> bool:
        for idx in range(self.count()):
            if self.itemText(idx) == text:
                return True
        return False

    def remove_text(self, text: str) -> None:
        idx = self.findText(text)
        self.removeItem(idx)
