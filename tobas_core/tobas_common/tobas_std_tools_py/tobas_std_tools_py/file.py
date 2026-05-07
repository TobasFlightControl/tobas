# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

import os.path as osp


def create_empty_file(file_path: str, exist_ok: bool = False) -> None:
    """指定されたパスにからのファイルを作成する．"""
    if osp.exists(file_path):
        if exist_ok:
            return
        else:
            raise RuntimeError(f"{file_path} already exists.")

    with open(file_path, "w"):
        pass  # ファイルに何も書き込まなければ空のファイルが作成される
