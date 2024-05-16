from overrides import override
from PyQt5.QtWidgets import QLayout, QWidget, QVBoxLayout

from ..widgets import ScrollArea


class ScrollableVBoxLayout(QVBoxLayout):
    def __init__(self) -> None:
        super().__init__()

        scroll_area = ScrollArea()
        super().addWidget(scroll_area)

        self._rows = QVBoxLayout()
        scroll_area.setLayout(self._rows)

    @override
    def addWidget(self, widget: QWidget) -> None:
        self._rows.addWidget(widget)

    @override
    def addLayout(self, layout: QLayout) -> None:
        self._rows.addLayout(layout)

    @override
    def addStretch(self) -> None:
        self._rows.addStretch()
