# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

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
            # Set the terminal to raw mode.
            tty.setraw(sys.stdin.fileno())

            # Wait for input.
            rlist, _, _ = select.select([sys.stdin], [], [], self._timeout)

            # Read the input.
            if rlist:
                # Read a single-character input.
                key = sys.stdin.read(1)

                # If the input is an escape character, read the remaining two characters.
                # FIXME: This waits when ESC alone is pressed.
                if key == KeyCode.ESC.value:
                    key += sys.stdin.read(2)

                return key
            else:
                return None
        finally:
            # Restore the terminal settings.
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)
