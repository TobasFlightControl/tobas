from typing import Tuple


def get_h_m_s(sec: int) -> Tuple[int, int, int]:
    """secをhour, min, secに変換する"""

    m, s = divmod(sec, 60)
    h, m = divmod(m, 60)
    return h, m, s
