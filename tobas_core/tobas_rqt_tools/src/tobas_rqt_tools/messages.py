from enum import Enum
from PyQt5.QtWidgets import QWidget, QMessageBox


class QMessageLevel(Enum):
    INFO = 0
    WARN = 1
    ERROR = 2


def q_info(parent: QWidget, msg: str) -> None:
    QMessageBox.information(parent, QMessageLevel.INFO.name, msg)


def q_warn(parent: QWidget, msg: str) -> None:
    QMessageBox.warning(parent, QMessageLevel.WARN.name, msg)


def q_error(parent: QWidget, msg: str) -> None:
    QMessageBox.critical(parent, QMessageLevel.ERROR.name, msg)


def q_info_named(parent: QWidget, name: str, msg: str) -> None:
    q_info(parent, f"[{name}] {msg}")


def q_warn_named(parent: QWidget, name: str, msg: str) -> None:
    q_warn(parent, f"[{name}] {msg}")


def q_error_named(parent: QWidget, name: str, msg: str) -> None:
    q_error(parent, f"[{name}] {msg}")


def yes_or_no(parent: QWidget, text: str, level: QMessageLevel) -> bool:
    """Yew-Noクエスチョンを表示し，その結果を取得する．"""
    msg_box = QMessageBox(parent)

    # メッセージレベルを設定
    if level is not None:
        if level == QMessageLevel.INFO:
            msg_box.setIcon(QMessageBox.Information)
            msg_box.setWindowTitle(QMessageLevel.INFO.name)
        elif level == QMessageLevel.WARN:
            msg_box.setIcon(QMessageBox.Warning)
            msg_box.setWindowTitle(QMessageLevel.WARN.name)
        elif level == QMessageLevel.WARN:
            msg_box.setIcon(QMessageBox.Critical)
            msg_box.setWindowTitle(QMessageLevel.ERROR.name)
        else:
            raise NotImplementedError()

    # テキストの設定
    msg_box.setText(text)

    # ボタンの設定
    # 配置は自動で決まる．明確な規則は無いが，全体でルールを統一することが大事: https://nanika.design/blog/1162/
    msg_box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
    msg_box.setDefaultButton(QMessageBox.No)

    # ユーザの返事を取得し，Yesの場合にTrueを返す
    return msg_box.exec() == QMessageBox.Yes
