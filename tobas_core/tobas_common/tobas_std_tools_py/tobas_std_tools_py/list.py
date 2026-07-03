# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

from typing import List, Union, Any


def is_unique(lst: List) -> bool:
    return len(lst) == len(set(lst))


def max_depth(lst: Union[List, Any]) -> int:
    """Return the maximum depth of a list, or its nesting level."""
    if isinstance(lst, List):
        if len(lst) == 0:
            # An empty list has depth 1.
            return 1
        else:
            # Recursively compute the maximum depth.
            return 1 + max(max_depth(item) for item in lst)
    else:
        # Non-list values have depth 0.
        return 0
