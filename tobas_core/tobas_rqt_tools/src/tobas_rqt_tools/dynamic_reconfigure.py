from typing import List


def get_param_config(configs: List[dict], name: str) -> dict:
    """dynamic_reconfigure.client.Client.get_descriptions()の返値からnameに対応するdictを返す．"""
    for config in configs:
        if config["name"] == name:
            return config
    raise RuntimeError(f"Failed to find '{name}'")
