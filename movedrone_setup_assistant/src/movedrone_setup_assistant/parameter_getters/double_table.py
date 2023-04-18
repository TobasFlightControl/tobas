import numpy as np
from numpy.typing import NDArray  # numpy >= 1.20
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from typing import List

from dh_rqt_tools.widgets import DoubleSpinBox

from .base import ParamGetterWidget
from ..utils import add_expanding_widget


class ParamGetterWidget_DoubleTable(ParamGetterWidget):

    BTN_HEIGHT = 30
    BTN_WIDTH = 80

    def __init__(
        self,
        param_name: str,
        description_text: str = None,
        labels: List[str] = [],
        col_width=100,
    ) -> None:
        super().__init__(param_name, description_text)

        self._data: List[List[DoubleSpinBox]]

        self._cols = QHBoxLayout()
        self._rows.addLayout(self._cols)

        self._add_row_btn = QPushButton("Add row")
        self._add_row_btn.setFixedSize(QSize(self.BTN_WIDTH, self.BTN_HEIGHT))
        self._add_row_btn.clicked.connect(self._add_row)
        self._cols.addWidget(self._add_row_btn)

        self._clear_btn = QPushButton("Clear")
        self._clear_btn.setFixedSize(QSize(self.BTN_WIDTH, self.BTN_HEIGHT))
        self._clear_btn.clicked.connect(self._clear)
        self._cols.addWidget(self._clear_btn)

        self._load_csv_btn = QPushButton("Load CSV")
        self._load_csv_btn.setFixedSize(QSize(self.BTN_WIDTH, self.BTN_HEIGHT))
        self._load_csv_btn.clicked.connect(self._load_csv)
        self._cols.addWidget(self._load_csv_btn)

        add_expanding_widget(self._cols)  # ボタンを左詰めにする

        self._table = QTableWidget(0, len(labels))
        self._table.setHorizontalHeaderLabels(labels)
        self._table.verticalHeader().setVisible(True)  # TODO: これで行番号が表示される？
        for c in range(self._table.columnCount()):
            self._table.setColumnWidth(c, col_width)
        self._rows.addWidget(self._table)

    def get(self) -> NDArray[np.float64]:
        pass  # TODO

    def set(self, src: NDArray[np.float64]) -> None:
        assert src.ndim == 2
        assert src.shape[1] == self._table.columnCount()

        # TODO

    @pyqtSlot()
    def _add_row(self) -> None:
        pass  # TODO

    @pyqtSlot()
    def _clear(self) -> None:
        pass  # TODO

    @pyqtSlot()
    def _load_csv(self) -> None:
        pass  # TODO
