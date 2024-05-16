from overrides import override
from PyQt5.QtWidgets import QStackedWidget

from ..utils import update_event_loop


class StackedWidget(QStackedWidget):
    """
    ===== QStackedWidgetとの違い =====
    - setCurrentIndexを安定化
    - 追加メソッド
    """

    @override
    def setCurrentIndex(self, index: int) -> None:
        # インデックスを更新
        super().setCurrentIndex(index)

        # Qtのイベントループを更新
        update_event_loop()

        # 画面を更新
        self.update()

    def clear(self) -> None:
        """全てのウィジェットを削除し，メモリを開放する．"""
        while self.count() > 0:
            widget = self.widget(0)
            self.removeWidget(widget)
            widget.deleteLater()
