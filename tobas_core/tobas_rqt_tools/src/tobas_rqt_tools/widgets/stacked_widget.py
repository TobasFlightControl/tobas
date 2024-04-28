from PyQt5.QtWidgets import QStackedWidget


class StackedWidget(QStackedWidget):
    """
    ===== QStackedWidgetとの違い =====
    - 追加メソッド
    """

    def clear(self) -> None:
        """全てのウィジェットを削除し，メモリを開放する．"""
        while self.count() > 0:
            widget = self.widget(0)
            self.removeWidget(widget)
            widget.deleteLater()
