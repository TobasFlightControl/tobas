from typing import override
from PyQt5.QtWidgets import QWidget, QScrollArea, QLayout


class ScrollArea(QScrollArea):
    """
    ===== QScrollAreaとの違い =====
    - デフォルトでスクロール可能
    - setLayoutをオーバーライド
    """

    def __init__(self) -> None:
        super().__init__()

        self.setWidgetResizable(True)

    @override
    def setLayout(self, layout: QLayout) -> None:
        # デフォルトのsetLayoutは親クラスであるQWidgetの名残であり，そのままでは使用できない
        # スクロールエリアに入れられるウィジェットは1つのみだから，Layoutを使うためには空のウィジェットを挟む必要がある
        inner_widget = QWidget()
        self.setWidget(inner_widget)
        inner_widget.setLayout(layout)
