from typing import List, Union, Any


def is_unique(lst: List) -> bool:
    return len(lst) == len(set(lst))


def max_depth(lst: Union[List, Any]) -> int:
    """リストの最大深さ，すなわち入れ子の階数を返す．"""
    if isinstance(lst, List):
        if len(lst) == 0:
            # 空のリストの深さは1
            return 1
        else:
            # 再帰的に最大深さを計算
            return 1 + max(max_depth(item) for item in lst)
    else:
        # リストでなければ深さは0
        return 0
