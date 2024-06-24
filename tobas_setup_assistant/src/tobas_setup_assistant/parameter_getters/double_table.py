import os.path as osp
import pandas as pd
import rospy
from overrides import override
from typing import List, Optional
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QPushButton, QFileDialog, QHBoxLayout

from tobas_property_tools_py.property_client import PropertyClient
from tobas_rqt_tools.widgets import DoubleSpinBox, TableWidget
from tobas_rqt_tools.messages import q_info, q_error
from tobas_tools_py.constants import PROPERTY_SERVER_GCS

from .base import ParamGetterWidget
from ..common import TITLE, PKG_NAME


class ParamGetterWidget_DoubleTable(ParamGetterWidget[List[List[float]]]):
    BTN_HEIGHT = 30
    BTN_WIDTH = 100
    DEFAULT_VALUE = 0.0
    DEFAULT_DECIMALS = 2

    data_changed = pyqtSignal()

    def __init__(self, param_name: str, labels: List[str], description_text: Optional[str] = None) -> None:
        super().__init__(param_name, description_text)

        # 最後に開かれたディレクトリの記録用
        self._property_client = PropertyClient(PROPERTY_SERVER_GCS, PKG_NAME)
        self._last_opened_dir_key = f'last_opened_dir/double_table/{param_name.lower().replace(" ", "_")}'

        self._labels = labels
        self._num_entry = len(labels)
        self._minimum = [-1e9] * self._num_entry
        self._maximum = [+1e9] * self._num_entry
        self._default = [self.DEFAULT_VALUE] * self._num_entry
        self._decimals = [self.DEFAULT_DECIMALS] * self._num_entry
        self._suffix = [""] * self._num_entry

        cols = QHBoxLayout()
        self._rows.addLayout(cols)

        self._add_row_btn = QPushButton("Add Row")
        self._add_row_btn.setFixedSize(self.BTN_WIDTH, self.BTN_HEIGHT)
        cols.addWidget(self._add_row_btn)

        self._delete_row_btn = QPushButton("Delete Row")
        self._delete_row_btn.setFixedSize(self.BTN_WIDTH, self.BTN_HEIGHT)
        cols.addWidget(self._delete_row_btn)

        self._clear_btn = QPushButton("Clear")
        self._clear_btn.setFixedSize(self.BTN_WIDTH, self.BTN_HEIGHT)
        cols.addWidget(self._clear_btn)

        self._load_csv_btn = QPushButton("Load CSV")
        self._load_csv_btn.setFixedSize(self.BTN_WIDTH, self.BTN_HEIGHT)
        cols.addWidget(self._load_csv_btn)

        cols.addStretch()  # ボタンを左詰めにする

        self._table = TableWidget(0, len(labels))
        self._table.setHorizontalHeaderLabels(labels)
        self._table.verticalHeader().setVisible(True)  # 行番号を表示
        self._rows.addWidget(self._table)

        # Connections
        self._add_row_btn.clicked.connect(self._add_row)
        self._delete_row_btn.clicked.connect(self._delete_row)
        self._clear_btn.clicked.connect(self._table.remove_all)
        self._load_csv_btn.clicked.connect(self._load_csv)

    @override
    def get(self) -> List[List[float]]:
        rows = self.count()
        res = [[0.0 for _ in range(self._num_entry)] for _ in range(rows)]

        for row in range(rows):
            for col in range(self._num_entry):
                cell: DoubleSpinBox = self._table.cellWidget(row, col)
                res[row][col] = cell.value()

        return res

    @override
    def set(self, src: List[List[float]]) -> None:
        assert self._is_valid_data(src), src

        self._table.remove_all()

        for row in range(len(src)):
            self._add_row()
            for col in range(len(src[row])):
                cell: DoubleSpinBox = self._table.cellWidget(row, col)
                cell.setValue(src[row][col])

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
        """Returns the number of samples."""
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
                f"Invalid column names: {df.columns.to_list()} \n" f"The required names are: {self._labels}",
            )
            return

        try:
            data = df.to_numpy(dtype=float)  # 強制的にfloatとして解釈
        except Exception as e:
            q_error(self.parent(), f"The data contains invalid data type. The error message is: {e}")
            return

        if not self.set(data.tolist()):
            return

        q_info(self.parent(), "Data is loaded successfully.")

    def _get_csv_file_path(self) -> str:
        res, last_opened_dir = self._property_client.get_string(self._last_opened_dir_key)
        if res < 0:
            rospy.logwarn(self._property_client.error_message())
            last_opened_dir = osp.expanduser("~")

        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file_path, _ = QFileDialog.getOpenFileName(self, TITLE, last_opened_dir, "CSV File (*.csv)", options=options)

        # 最後に開かれたパスを保存
        if file_path != "":
            if self._property_client.set_string(self._last_opened_dir_key, osp.dirname(file_path)) < 0:
                rospy.logerr(self._property_client.error_message())
            if self._property_client.save() < 0:
                rospy.logerr(self._property_client.error_message())

        return file_path

    def _is_valid_data(self, src: List[List[float]]) -> bool:
        for row in src:
            if len(row) != self._num_entry:
                return False

            for col in range(self._num_entry):
                value = row[col]
                if value < self._minimum[col] or self._maximum[col] < value:
                    q_error(self, f"{value}[{self._suffix[col]}] is invalid for {self._labels[col]}.")
                    return False

        return True
