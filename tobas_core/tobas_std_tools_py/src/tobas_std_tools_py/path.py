import os.path as osp


def get_basename_without_extension(path: str) -> str:
    """
    パスの拡張子を除くベース名を返す．

    Examples
    ----------
    /hoge/fuga/piyo.ext -> piyo
    """
    assert not path.endswith("/")
    return osp.splitext(osp.basename(path))[0]
