from typing import List
from configparser import ConfigParser
import numpy as np
from numpy.typing import NDArray  # numpy >= 1.20
import pandas as pd
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.widgets import DoubleSpinBox, add_expanding_widget
from dh_rqt_tools.messages import q_info, q_error

from .base import ParamGetterWidget
from ..common import *


class ParamGetterWidget_DoubleTable(ParamGetterWidget):

    BTN_HEIGHT = 30
    BTN_WIDTH = 100
    DEFAULT_VALUE = 0.
    DEFAULT_DECIMALS = 2

    data_changed = pyqtSignal()

    def __init__(
        self,
        param_name: str,
        labels: List[str],
        description_text: str = None,
    ) -> None:
        super().__init__(param_name, description_text)

        # 最後に開かれたディレクトリの記録用
        self._config = ConfigParser()
        self._path_key = f'last_opened_dir/double_table/{param_name.lower().replace(" ", "_")}'

        self._labels = labels
        self._num_entry = len(labels)
        self._minimum = [-1e+9] * self._num_entry
        self._maximum = [+1e+9] * self._num_entry
        self._default = [self.DEFAULT_VALUE] * self._num_entry
        self._decimals = [self.DEFAULT_DECIMALS] * self._num_entry
        self._suffix = [""] * self._num_entry

        self._cols = QHBoxLayout()
        self._rows.addLayout(self._cols)

        self._add_row_btn = QPushButton("Add row")
        self._add_row_btn.setFixedSize(QSize(self.BTN_WIDTH, self.BTN_HEIGHT))
        self._add_row_btn.clicked.connect(self._add_row)
        self._cols.addWidget(self._add_row_btn)

        self._delete_row_btn = QPushButton("Delete row")
        self._delete_row_btn.setFixedSize(QSize(self.BTN_WIDTH, self.BTN_HEIGHT))
        self._delete_row_btn.clicked.connect(self._delete_row)
        self._cols.addWidget(self._delete_row_btn)

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
        self._table.verticalHeader().setVisible(True)  # 行番号を表示
        self._rows.addWidget(self._table)

    def get(self) -> NDArray[np.float64]:
        """
        Returns
        -------
        NDArray[np.float64]
            shape = (num_data, num_entry)
        """
        rows = self.count()
        res = np.empty((rows, self._num_entry))

        for row in range(rows):
            for col in range(self._num_entry):
                cell: DoubleSpinBox = self._table.cellWidget(row, col)
                res[row, col] = cell.value()

        return res

    def set(self, src: NDArray[np.float64]) -> bool:
        """
        Parameters
        ----------
        src : NDArray[np.float64]
            shape = (num_data, num_entry)

        Returns
        ----------
        bool
            success or failure
        """
        if not self._is_valid_data(src):
            return False

        self._clear()
        for row in range(src.shape[0]):
            self._add_row()
            for col in range(src.shape[1]):
                cell: DoubleSpinBox = self._table.cellWidget(row, col)
                cell.setValue(src[row, col])

        return True

    def set_minimum(self, minimum: List[float]) -> None:
        assert len(minimum) == self._num_entry
        for min_, max_ in zip(minimum, self._maximum):
            assert min_ <= max_

        self._minimum = minimum

    def set_maximum(self, maximum: List[float]) -> None:
        assert len(maximum) == self._num_entry
        for min_, max_ in zip(self._minimum, maximum):
            assert min_ <= max_

        self._maximum = maximum

    def set_default(self, default: List[float]) -> None:
        assert len(default) == self._num_entry
        for min_, max_, def_ in zip(self._minimum, self._maximum, default):
            assert min_ <= def_ <= max_

        self._default = default

    def set_decimals(self, decimals: List[int]) -> None:
        assert len(decimals) == self._num_entry
        for decimal in decimals:
            assert decimal >= 0

        self._decimals = decimals

    def set_suffix(self, suffix: List[str]) -> None:
        assert len(suffix) == self._num_entry

        self._suffix = suffix

    def set_fixed_height(self, height: int) -> None:
        assert height > 0

        self._table.setFixedHeight(height)

    def set_column_width(self, width: int) -> None:
        assert width > 0

        for col in range(self._num_entry):
            self._table.setColumnWidth(col, width)

    def count(self) -> int:
        """ Returns the number of samples. """
        return self._table.rowCount()

    @pyqtSlot()
    def _add_row(self) -> None:
        rows = self.count()
        self._table.insertRow(rows)

        for col in range(self._num_entry):
            cell = DoubleSpinBox()
            cell.setMinimum(self._minimum[col])
            cell.setMaximum(self._maximum[col])
            cell.setValue(self._default[col])
            cell.setDecimals(self._decimals[col])
            cell.setSuffix(self._suffix[col])
            cell.valueChanged.connect(lambda: self.data_changed.emit())
            self._table.setCellWidget(rows, col, cell)

    @pyqtSlot()
    def _delete_row(self) -> None:
        row = self._table.currentRow()
        if row < 0:
            return
        self._table.removeRow(row)

    @pyqtSlot()
    def _clear(self) -> None:
        rows = self.count()
        for _ in range(rows):
            self._table.removeRow(0)

    @pyqtSlot()
    def _load_csv(self) -> None:
        file_path = self._get_csv_file_path()
        if file_path == "":  # Cancelの場合
            return

        df = pd.read_csv(file_path, sep=",")

        try:
            df = df[self._labels]  # labelsに合致する列を抽出．labelsと同じ順になる．
            assert df.columns.to_list() == self._labels
        except Exception as e:
            q_error(
                self.parent(),
                f'Invalid column names: {df.columns.to_list()} \n'
                f'The required names are: {self._labels}'
            )
            return

        try:
            data = df.to_numpy(dtype=float)
        except Exception as e:
            q_error(
                self.parent(),
                f'The data contains invalid data type. The error message is: {e}'
            )
            return

        if not self.set(data):
            return

        q_info(self.parent(), "Data is successfully loaded.")

    def _get_csv_file_path(self) -> str:
        self._config.read(CONFIG_PATH)
        last_opened_dir = self._config.get(DEFAULT, self._path_key, fallback=osp.expanduser("~"))

        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file_path, _ = QFileDialog.getOpenFileName(
            self, TITLE, last_opened_dir, "CSV File (*.csv)", options=options
        )

        # 最後に開かれたパスを保存
        if file_path != "":
            self._config[DEFAULT][self._path_key] = osp.dirname(file_path)
            with open(CONFIG_PATH, "w") as f:
                self._config.write(f)

        return file_path

    def _is_valid_data(self, src: NDArray[np.float64]) -> bool:
        assert src.ndim == 2
        assert src.shape[1] == self._num_entry

        for row in range(src.shape[0]):
            for col in range(self._num_entry):
                val = src[row, col]
                if not self._minimum[col] <= val <= self._maximum[col]:
                    q_error(
                        self.parent(),
                        f'{val}[{self._suffix[col]}] is invalid for {self._labels[col]}.',
                    )
                    return False

        return True
