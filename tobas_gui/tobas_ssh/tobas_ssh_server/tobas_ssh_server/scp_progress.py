# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

import time
from typing import Callable


class RecursiveScpProgress:
    def __init__(self, callback: Callable[[int], None], update_interval: float = 0.1) -> None:
        self._callback = callback
        self._update_interval = update_interval

        self._transferred = 0  # Transferred size for the entire directory.
        self._last_update_time = 0.0  # Time when the user callback was last called.

        # `scp` `sent` is the number of bytes transferred within that file,
        # so keep the previous value to compute per-file differences.
        self._last_sent_by_file: dict[str, int] = {}

    def __call__(self, filename: str, size: int, sent: int) -> None:
        # `size=1` and `sent=1` are passed for empty files.
        # The actual data size is 0 bytes, so do not add it to the total transferred size.
        if filename not in self._last_sent_by_file and size == 1 and sent == 1:
            self._last_sent_by_file[filename] = sent
            self._callback(self._transferred)
            return

        # Add the difference from the previous transferred size.
        previous_sent = self._last_sent_by_file.get(filename, 0)
        delta = max(0, sent - previous_sent)
        self._transferred += delta

        # Save the total transferred size of the current file.
        self._last_sent_by_file[filename] = sent

        # Call the user callback at a fixed interval.
        now = time.monotonic()
        if now - self._last_update_time >= self._update_interval:
            self._callback(self._transferred)
            self._last_update_time = now
