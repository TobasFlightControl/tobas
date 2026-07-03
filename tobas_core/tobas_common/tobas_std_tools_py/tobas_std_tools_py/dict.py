# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

from copy import deepcopy


def concatenate(a: dict, b: dict) -> dict:
    """Merge two nested dictionaries."""
    res = deepcopy(a)

    for key, b_val in b.items():
        if key in a.keys():
            a_val = a[key]
            if isinstance(a_val, dict) and isinstance(b_val, dict):
                res[key] = concatenate(a_val, b_val)
            elif not isinstance(a_val, dict) and not isinstance(b_val, dict):
                if a_val != b_val:
                    raise RuntimeError(f'Values for key "{key}" conflict: "{a_val}" vs "{b_val}"')
            else:
                raise RuntimeError(f'Different types for key "{key}"')
        else:
            res[key] = b_val

    return res
