from PyQt5.QtWidgets import QTableWidget


class TableWidget(QTableWidget):
    """
    ===== QTableWidgetとの違い =====
    - 追加メソッド
    """

    def remove_all(self) -> None:
        """全ての行を削除する．clearとは異なり，内容に加えセルまで削除する．"""
        while self.rowCount() > 0:
            self.removeRow(0)
