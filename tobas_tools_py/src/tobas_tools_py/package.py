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
    """PC上のTobasメタパッケージの絶対パスを返す．"""
    return osp.join(tbs_path, get_tbs_meta_name(tbs_path))


def get_tbs_config_path(tbs_path: str) -> str:
    """PC上のTobas設定パッケージの絶対パスを返す．"""
    return osp.join(tbs_path, get_tbs_config_name(tbs_path))


def get_tbs_user_path(tbs_path: str) -> str:
    """PC上のTobasユーザパッケージの絶対パスを返す．"""
    return osp.join(tbs_path, get_tbs_user_name(tbs_path))


def get_tbsdrn_path(tbs_path: str) -> str:
    """PC上の drone.tbsdrn の絶対パスを返す．"""
    return osp.join(get_tbs_config_path(tbs_path), "config", "drone.tbsdrn")


def get_modified_urdf_path(tbs_path: str) -> str:
    """PC上の drone.xacro の絶対パスを返す．"""
    return osp.join(get_tbs_config_path(tbs_path), "urdf", "drone.xacro")


def get_mesh_path(tbs_path: str) -> str:
    """PC上の meshディレクトリの絶対パスを返す．"""
    return osp.join(get_tbs_config_path(tbs_path), "mesh")


def get_dynamic_params_path(tbs_path: str) -> str:
    """PC上の dynamic_params.yaml の絶対パスを返す．"""
    return osp.join(get_tbs_config_path(tbs_path), "config", "dynamic_params.yaml")


def get_settings_path(tbs_path: str) -> str:
    """PC上のバックアップ用設定ファイルの絶対パスを返す．"""
    return osp.join(get_tbs_config_path(tbs_path), "backup", "settings.yaml")


def get_original_urdf_path(tbs_path: str) -> str:
    """PC上のバックアップ用オリジナルURDFの絶対パスを返す．"""
    return osp.join(get_tbs_config_path(tbs_path), "backup", "original.urdf")
