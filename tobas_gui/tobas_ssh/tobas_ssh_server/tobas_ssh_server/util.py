# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

import os
from pathlib import Path
from typing import List


def is_under(path: str, base: str, *, follow_symlinks: bool = True) -> bool:
    """
    Determine whether `path` exists under `base`, meaning it points to a file or directory below it.
    If `follow_symlinks=True`, check using the real path after resolving symbolic links.
    """
    p = Path(path)
    b = Path(base)

    if follow_symlinks:
        # Use `strict=False` to normalize as much as possible even for paths that do not exist.
        p = p.resolve(strict=False)
        b = b.resolve(strict=False)
    else:
        p = p.absolute()
        b = b.absolute()

    try:
        p.relative_to(b)
        return True
    except ValueError:
        return False


def is_under_any(path: str, bases: List[str], *, follow_symlinks: bool = True) -> bool:
    for base in bases:
        if is_under(path, base, follow_symlinks=follow_symlinks):
            return True
    return False


def get_local_tree_size(_path: str, _exclude_dirs: List[str] = []) -> int:
    path = Path(_path)

    if path.is_file():
        return path.stat(follow_symlinks=False).st_size

    if not path.is_dir():
        return 0

    total_size = 0

    for root, _, filenames in os.walk(path):
        if is_under_any(root, _exclude_dirs):
            continue
        for filename in filenames:
            path = Path(root) / filename
            try:
                total_size += path.stat(follow_symlinks=False).st_size
            except OSError:
                pass

    return total_size
