import os
import os.path as osp

from .constants import PKG_EXTENSION
from .package import get_tbs_meta_name, get_tbs_config_path, get_tbs_user_path


def kill_gazebo() -> bool:
    return os.system("rosnode kill /gazebo /gazebo_gui || true && killall -9 gzserver gzclient") == 0


def build_tobas_package(tbs_path: str) -> bool:
    assert tbs_path.endswith(PKG_EXTENSION)
    assert osp.isdir(tbs_path)

    os.chdir(tbs_path)
    meta_name = get_tbs_meta_name(tbs_path)
    return os.system(f"catkin build {meta_name}") == 0


def source_tobas_package(tbs_path: str) -> None:
    assert tbs_path.endswith(PKG_EXTENSION)
    assert osp.isdir(tbs_path)

    config_path = get_tbs_config_path(tbs_path)
    user_path = get_tbs_user_path(tbs_path)
    os.environ["ROS_PACKAGE_PATH"] = f'{config_path}:{user_path}:{os.environ["ROS_PACKAGE_PATH"]}'
