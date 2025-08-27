import sys
import select
import termios
import tty
from typing import Union
from enum import Enum


class KeyCode(Enum):
    ENTER = "\x0a"
    ESC = "\x1b"
    UP = "\x1b[A"
    DOWN = "\x1b[B"
    RIGHT = "\x1b[C"
    LEFT = "\x1b[D"
    CTRL_C = "\x03"


class KeyboardReader:
    def __init__(self, timeout: float) -> None:
        assert timeout >= 0.0

        self._timeout = timeout
        self._settings = termios.tcgetattr(sys.stdin)

    def read(self) -> Union[str, None]:
        try:
            # ターミナルをrawモードに設定
            tty.setraw(sys.stdin.fileno())

            # 入力の待機
            rlist, _, _ = select.select([sys.stdin], [], [], self._timeout)

            # 入力を取得
            if rlist:
                # 1文字入力を読み取る
                key = sys.stdin.read(1)

                # エスケープ文字だったら残りの2文字を読み取る
                # FIXME: このやり方だとESC単体が押された場合に待機してしまう
                if key == KeyCode.ESC.value:
                    key += sys.stdin.read(2)

                return key
            else:
                return None
        finally:
            # ターミナル設定を元に戻す
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)
