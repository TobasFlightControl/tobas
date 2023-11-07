import os.path as osp
import rospy
import csv
from typing import List
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from mavros_msgs.srv import ParamSet, ParamSetRequest, ParamSetResponse

from dh_rqt_tools.widgets import *
from dh_rqt_tools.layouts import FormLayout
from dh_rqt_tools.path import get_proj_path
from dh_rqt_tools.messages import q_info, q_error

from .common import *


class DynamicConfigurationWidget(MainWidget):
    LAST_OPENED_DIR = "last_opened_dir"
    SECTIONS = ["ATC", "PSC"]

    def __init__(self) -> None:
        super().__init__(CONFIG_PATH, DEFAULT)

        self._param_set_sc = rospy.ServiceProxy(PARAM_SET_SRV_NAME, ParamSet)

        proj_path = get_proj_path()
        icon_path = osp.join(proj_path, "resources/icon.png")
        self.setWindowIcon(QIcon(icon_path))
        self.setWindowTitle(TITLE)

        rows = QVBoxLayout()
        self.setLayout(rows)

        cols = QHBoxLayout()
        rows.addLayout(cols)

        self._load_button = QPushButton("Load")
        cols.addWidget(self._load_button)

        self._save_button = QPushButton("Save")
        cols.addWidget(self._save_button)

        self._form = FormLayout()
        form_widget = QWidget()
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        form_widget.setLayout(self._form)
        scroll_area.setWidget(form_widget)
        rows.addWidget(scroll_area)

        self.define_connections()

    def define_connections(self) -> None:
        self._load_button.clicked.connect(self._on_load_button_clicked)
        self._save_button.clicked.connect(self._on_save_button_clicked)

    @pyqtSlot()
    def _on_load_button_clicked(self) -> None:
        # パラメータサーバが利用可能化チェック
        try:
            rospy.wait_for_service(PARAM_SET_SRV_NAME, WAIT_FOR_SERVICE)
        except rospy.ROSException as e:
            q_error(self, "ArduPilot parameter server is not available.")
            return

        # 前回開いたパスを取得
        self._config.read(CONFIG_PATH)  # 排他処理のためにこの関数内でRead & Write
        last_opened_dir = self._config.get(
            DEFAULT,
            self.LAST_OPENED_DIR,
            fallback=osp.expanduser("~"),
        )

        # paramsのパスを取得
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            TITLE,
            last_opened_dir,
            "Parameter Files (*.params)",
            options=options,
        )

        # キャンセルの場合は何もせずに終了 (そうしないと空文字が設定されてしまう)
        if file_path == "":
            return

        # ユーザが開いたディレクトリを保存
        # closeEvent()に書くと強制終了時に呼ばれないため，ファイル読み込み時に同時に保存する
        self._config[DEFAULT][self.LAST_OPENED_DIR] = osp.dirname(file_path)
        with open(CONFIG_PATH, "w") as f:
            self._config.write(f)

        # フォームを初期化
        self._form.clear()

        # パラメータを読み込む
        self._load_params(file_path)

    @pyqtSlot()
    def _on_save_button_clicked(self) -> None:
        pass  # TODO

    def _load_params(self, file_path: str) -> None:
        fail_params: List[str] = []  # ロードに失敗したパラメータ
        req = ParamSetRequest()

        with open(file_path, "r") as file:
            # CSVリーダーを使用してタブ区切りでデータを解析する
            reader = csv.reader(file, delimiter="\t")

            for row in reader:
                # 空行とコメント行を無視
                if not row or row[0].startswith("#"):
                    continue

                # 行からパラメータデータを抽出
                try:
                    vehicle_id, component_id, param_name, value, param_type = row
                except ValueError as e:
                    rospy.logerr(f"Error parsing line: {row}. Error: {e}")
                    continue

                # 使用するセクションでない場合はスキップ
                section = param_name.split("_")[0]
                if section not in self.SECTIONS:
                    continue

                # ArduPilot SITLに反映
                req.param_id = param_name
                try:
                    res: ParamSetResponse = self._param_set_sc(req)
                except rospy.ServiceException as e:
                    rospy.logerr(f"Failed to set {param_name}: {e}")
                    fail_params.append(param_name)
                    continue

                if not res.success:
                    rospy.logerr(f"Failed to set {param_name}.")
                    fail_params.append(param_name)
                    continue

                if int(param_type) == 9:
                    # パラメータをフォームに追加
                    spin_box = DoubleSpinBox()
                    spin_box.setDecimals(FLOAT_DECIMALS)
                    spin_box.setValue(float(value))
                    self._form.addRow(QLabel(param_name), spin_box)

                    # サービスリクエストを埋める
                    req.value.integer = 0
                    req.value.real = float(value)
                else:
                    # パラメータをフォームに追加
                    spin_box = SpinBox()
                    spin_box.setValue(int(value))
                    self._form.addRow(QLabel(param_name), spin_box)

                    # サービスリクエストを埋める
                    req.value.integer = int(value)
                    req.value.real = 0.0

        if len(fail_params) == 0:
            q_info(self, "Parameters are successfully loaded.")
        else:
            fail_params_str = "\n".join(fail_params)
            q_error(self, f"Failed to set following parameters:\n\n{fail_params_str}")
