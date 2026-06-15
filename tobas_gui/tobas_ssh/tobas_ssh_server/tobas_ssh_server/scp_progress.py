# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

import time
from typing import Callable


class RecursiveScpProgress:
    def __init__(self, callback: Callable[[int], None], update_interval: float = 0.0) -> None:
        self._callback = callback
        self._update_interval = update_interval

        self._transferred = 0  # ディレクトリ全体の転送量
        self._last_update_time = 0.0  # 最後にユーザコールバックを呼んだ時刻

        # scpのsentはそのファイル内での転送済みbyte数だから，ファイルごとの差分を取るために前回値を保持する．
        self._last_sent_by_file: dict[str, int] = {}

    def __call__(self, filename: str, size: int, sent: int) -> None:
        # 空のファイルに対してはsize=1,sent=1が渡される
        # 実データは0 byteだから全体転送量には加算しない
        if filename not in self._last_sent_by_file and size == 1 and sent == 1:
            self._last_sent_by_file[filename] = sent
            self._callback(self._transferred)
            return

        # 前回の転送量からの差分を加算
        previous_sent = self._last_sent_by_file.get(filename, 0)
        delta = max(0, sent - previous_sent)
        self._transferred += delta

        # 現在のファイルの総転送量を保存
        self._last_sent_by_file[filename] = sent

        # 一定周期でユーザコールバックを呼ぶ
        now = time.monotonic()
        if now - self._last_update_time >= self._update_interval:
            self._callback(self._transferred)
            self._last_update_time = now
