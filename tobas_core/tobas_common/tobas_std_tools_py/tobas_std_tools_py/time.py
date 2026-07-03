# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

from typing import Tuple


def get_h_m_s(sec: int) -> Tuple[int, int, int]:
    """Convert `sec` to hour, minute, and second."""

    m, s = divmod(sec, 60)
    h, m = divmod(m, 60)
    return h, m, s
