def deep_connect(a: dict, b: dict) -> dict:
    """葉が重複しない2層までの2つの辞書を結合する"""

    res = a.copy()
    for key1, val1 in b.items():
        if key1 in res.keys():
            if isinstance(val1, dict):
                for key2, val2 in val1.items():
                    if key2 in res[key1].keys():
                        if isinstance(val2, dict):
                            raise ValueError("Connection of dicts with a depth of more than 2 is not supported.")
                        else:
                            raise ValueError(f'Both dicts have the same key "{key1}/{key2}"')
                    else:
                        res[key1][key2] = val2
            else:
                raise ValueError(f'Both dicts have the same key "{key1}"')
        else:
            res[key1] = val1
    return res
