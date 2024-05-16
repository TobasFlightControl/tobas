import os
import sys
import signal
from typing import Type
from PyQt5.QtCore import Qt, QObject, QEventLoop, QTimer
from PyQt5.QtWidgets import QWidget, QBoxLayout, QVBoxLayout, QHBoxLayout, QMessageBox


def block_signals_rec(obj: QObject, block: bool) -> None:
    """子ウィジェットを再帰的に走査し，全てのシグナルをブロックする．"""
    obj.blockSignals(block)
    for child in obj.children():
        block_signals_rec(child, block)


def place_center(widget: QWidget, rows: QVBoxLayout) -> None:
    """ウィジェットをレイアウトの中央に配置する．"""
    cols = QHBoxLayout()
    cols.addWidget(widget)
    cols.setAlignment(widget, Qt.AlignCenter)
    rows.addLayout(cols)


def create_fixed_width_vboxlayout(width: int, parent: QBoxLayout) -> QVBoxLayout:
    widget = QWidget()
    widget.setFixedWidth(width)
    parent.addWidget(widget)

    res = QVBoxLayout()
    widget.setLayout(res)

    return res


def create_fixed_height_hboxlayout(height: int, parent: QBoxLayout) -> QHBoxLayout:
    widget = QWidget()
    widget.setFixedHeight(height)
    parent.addWidget(widget)

    res = QHBoxLayout()
    widget.setLayout(res)

    return res


def qsleep(msec: int) -> None:
    loop = QEventLoop()
    QTimer.singleShot(msec, loop.quit)
    loop.exec_()


def update_event_loop() -> None:
    """Qtのイベントループを僅かに進める．"""
    qsleep(0)


def handle_unexpected_exception(exc_type: Type[BaseException], exc_value: BaseException, exc_traceback):
    """未処理の例外をキャッチし，プロセスを強制終了する．"""
    QMessageBox.critical(
        None,
        "Fatal",
        "An unexpected error occurred. "
        "The application will shutdown soon. "
        "Please report this issue to the developers.\n"
        "---\n"
        f"{exc_value}",
    )
    sys.__excepthook__(exc_type, exc_value, exc_traceback)  # 元の例外ハンドラを呼び出す
    os.kill(os.getpid(), signal.SIGINT)  # 強制終了
