import os.path as osp
import rospy
import rospkg
import csv
from datetime import datetime
from typing import List
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import ParamSet, ParamSetRequest, ParamSetResponse

from tobas_rqt_tools.widgets import *
from tobas_rqt_tools.layouts import FormLayout
from tobas_rqt_tools.messages import q_info, q_error

from .common import *
from .param_holders import *


class DynamicConfigurationWidget(MainWidget):
    BUTTON_HEIGHT = 30
    LAST_OPENED_DIR = "last_opened_dir"
    SECTIONS = ["ATC", "PSC"]

    def __init__(self) -> None:
        super().__init__(PKG_NAME)

        icon_path = osp.join(rospkg.RosPack().get_path(PKG_NAME), "resources/icon.png")
        self.setWindowIcon(QIcon(icon_path))
        self.setWindowTitle(TITLE)

        rows = QVBoxLayout()
        self.setLayout(rows)

        cols = QHBoxLayout()
        rows.addLayout(cols)

        self._load_button = QPushButton("Load")
        self._load_button.setFixedHeight(self.BUTTON_HEIGHT)
        self._load_button.setEnabled(False)
        cols.addWidget(self._load_button)

        self._save_button = QPushButton("Save")
        self._save_button.setFixedHeight(self.BUTTON_HEIGHT)
        self._save_button.setEnabled(False)
        cols.addWidget(self._save_button)

        scroll_area = ScrollArea()
        scroll_area.setWidgetResizable(True)
        rows.addWidget(scroll_area)

        self._form = FormLayout()
        scroll_area.setLayout(self._form)

        self.define_connections()

        self._params = dict()
        self._local_pose_sub = rospy.Subscriber(
            "mavros/local_position/pose", PoseStamped, self._local_pose_cb, queue_size=1
        )
        self._param_set_sc = rospy.ServiceProxy(PARAM_SET_SRV_NAME, ParamSet)

    def define_connections(self) -> None:
        self._load_button.clicked.connect(self._on_load_button_clicked)
        self._save_button.clicked.connect(self._on_save_button_clicked)

    def _load_params(self, file_path: str) -> None:
        fail_params: List[str] = []  # ロードに失敗したパラメータ
        req = ParamSetRequest()

        with open(file_path, "r") as f:
            # CSVリーダーを使用してタブ区切りでデータを解析する
            reader = csv.reader(f, delimiter="\t")

            for row in reader:
                # 空行とコメント行を無視
                if not row or row[0].startswith("#"):
                    continue

                # 行からパラメータデータを抽出
                try:
                    vehicle_id, component_id, name, value, type_ = row
                except ValueError as e:
                    q_error(self, f"Failed to parse line: {row}. Error: {e}")
                    return

                # パラメータを辞書に保存
                self._params[name] = {
                    "vehicle_id": vehicle_id,
                    "component_id": component_id,
                    "name": name,
                    "value": value,
                    "type": type_,
                }

                # 使用するセクションでない場合はスキップ
                section = name.split("_")[0]
                if section not in self.SECTIONS:
                    continue

                if int(type_) == 9:
                    # パラメータをフォームに追加
                    param = FloatParam(name)
                    param.setDecimals(FLOAT_DECIMALS)
                    param.setSingleStep(10 ** (-FLOAT_DECIMALS))
                    param.setValue(float(value))
                    param.value_changed.connect(self._on_float_param_changed)
                    self._form.addRow(QLabel(name), param)

                    # サービスリクエストを埋める
                    req.value.integer = 0
                    req.value.real = float(value)

                else:
                    # パラメータをフォームに追加
                    param = IntParam(name)
                    param.setValue(int(value))
                    param.value_changed.connect(self._on_int_param_changed)
                    self._form.addRow(QLabel(name), param)

                    # サービスリクエストを埋める
                    req.value.integer = int(value)
                    req.value.real = 0.0

                # ArduPilot SITLに反映
                req.param_id = name
                try:
                    res: ParamSetResponse = self._param_set_sc(req)
                except rospy.ServiceException as e:
                    rospy.logerr(f"Failed to set {name}: {e}")
                    self._form.removeRow(self._form.rowCount() - 1)  # 先程追加した最後の行を削除
                    fail_params.append(name)
                    continue

                if not res.success:
                    rospy.logerr(f"Failed to set {name}.")
                    fail_params.append(name)
                    continue

        if len(fail_params) == 0:
            q_info(self, "Parameters are loaded successfully.")
        else:
            fail_params_str = "\n".join(fail_params)
            q_error(self, f"Failed to set following parameters:\n{fail_params_str}")

    def _send_param_set_request(self, req: ParamSetRequest) -> bool:
        try:
            res: ParamSetResponse = self._param_set_sc(req)
        except rospy.ServiceException as e:
            q_error(self, f"Failed to set {req.param_id}: {e}")
            return False

        if not res.success:
            q_error(self, f"Failed to set {req.param_id}.")
            return False

        rospy.loginfo(f"{req.param_id} is updated.")
        return True

    def _local_pose_cb(self, _: PoseStamped) -> None:
        self._load_button.setEnabled(True)
        self._save_button.setEnabled(True)
        rospy.loginfo("Dynamic configuration is ready.")
        self._local_pose_sub.unregister()

    @pyqtSlot()
    def _on_load_button_clicked(self) -> None:
        # パラメータサーバが利用可能化チェック
        try:
            self._param_set_sc.wait_for_service(WAIT_FOR_SERVER)
        except rospy.ROSException as e:
            q_error(self, "ArduPilot parameter server is not available.")
            return

        # 前回開いたパスを取得
        self._config.read(self._config_path)  # 排他処理のためにこの関数内でRead & Write
        last_opened_dir = self._config.get(
            self._section,
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
        self._config[self._section][self.LAST_OPENED_DIR] = osp.dirname(file_path)
        with open(self._config_path, "w") as f:
            self._config.write(f)

        # フォームと辞書を初期化
        self._form.clear()
        self._params.clear()

        # パラメータを読み込む
        self._load_params(file_path)

    @pyqtSlot()
    def _on_save_button_clicked(self) -> None:
        # 前回開いたパスを取得
        self._config.read(self._config_path)  # 排他処理のためにこの関数内でRead & Write
        last_opened_dir = self._config.get(
            self._section,
            self.LAST_OPENED_DIR,
            fallback=osp.expanduser("~"),
        )

        # paramsのパスを取得
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file_path, _ = QFileDialog.getSaveFileName(  # 上書きするかどうかの確認も自動でやってくれる
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
        self._config[self._section][self.LAST_OPENED_DIR] = osp.dirname(file_path)
        with open(self._config_path, "w") as f:
            self._config.write(f)

        # TSVファイルを保存
        try:
            with open(file_path, "w") as f:
                # Write header
                f.write(f"# This file was automatically generated by {PKG_NAME} on {datetime.now()}\n")
                f.write(f"#\n")
                f.write("# Vehicle-Id Component-Id Name Value Type\n")

                # Write parameters
                for p in self._params.values():
                    f.write(f'{p["vehicle_id"]}\t{p["component_id"]}\t{p["name"]}\t{p["value"]}\t{p["type"]}\n')

            q_info(self, "Parameters are saved successfully.")
        except Exception as e:
            q_error(self, f"An error occurred while saving the file: {e}")

    @pyqtSlot(str, int)
    def _on_int_param_changed(self, name: str, value: int):
        req = ParamSetRequest()
        req.param_id = name
        req.value.integer = value

        if self._send_param_set_request(req):
            self._params[name]["value"] = str(value)
        else:
            pass  # TODO: 設定に失敗したパラメータをリセット

    @pyqtSlot(str, float)
    def _on_float_param_changed(self, name: str, value: float):
        req = ParamSetRequest()
        req.param_id = name
        req.value.real = value

        if self._send_param_set_request(req):
            self._params[name]["value"] = str(value)
        else:
            pass  # TODO: 設定に失敗したパラメータをリセット
