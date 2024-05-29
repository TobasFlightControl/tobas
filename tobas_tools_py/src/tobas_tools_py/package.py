import os.path as osp


from .constants import PKG_EXTENSION


def get_tbs_name(tbs_path: str) -> str:
    assert tbs_path.endswith(PKG_EXTENSION)
    return osp.splitext(osp.basename(tbs_path))[0]


def get_tbs_meta_name(tbs_path: str) -> str:
    return get_tbs_name(tbs_path)


def get_tbs_config_name(tbs_path: str) -> str:
    return get_tbs_name(tbs_path) + "_config"


def get_tbs_user_name(tbs_path: str) -> str:
    return get_tbs_name(tbs_path) + "_user"


def get_tbs_meta_path(tbs_path: str) -> str:
    return osp.join(tbs_path, get_tbs_meta_name(tbs_path))


def get_tbs_config_path(tbs_path: str) -> str:
    return osp.join(tbs_path, get_tbs_config_name(tbs_path))


def get_tbs_user_path(tbs_path: str) -> str:
    return osp.join(tbs_path, get_tbs_user_name(tbs_path))


def get_tbsdrn_path(tbs_path: str) -> str:
    return osp.join(get_tbs_config_path(tbs_path), "config", "drone.tbsdrn")


def get_urdf_path(tbs_path: str) -> str:
    return osp.join(get_tbs_config_path(tbs_path), "urdf", "drone.xacro")
