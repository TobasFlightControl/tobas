from typing import override
from typing import Iterator
from PyQt5.QtWidgets import QWidget, QStackedWidget

from ..utils import update_event_loop


class StackedWidget(QStackedWidget):
    """
    ===== QStackedWidgetとの違い =====
    - イテレータを定義
    - setCurrentIndexを安定化
    - 追加メソッド
    """

    def __iter__(self) -> Iterator[QWidget]:
        for i in range(self.count()):
            yield self.widget(i)

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
