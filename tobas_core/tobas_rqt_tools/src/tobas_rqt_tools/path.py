import os.path as osp
import rospkg
from urllib import parse
from glob import glob
from typing import List


def resolve_uri(uri: str) -> str:
    """URDF中のファイルの絶対パスを返す．"""
    if uri.startswith("package://"):
        pkg_name = uri.split("package://")[1].split("/")[0]
        rest_of_path = uri.split(f"package://{pkg_name}/")[1]
        pkg_path = rospkg.RosPack().get_path(pkg_name)
        return osp.join(pkg_path, rest_of_path)
    elif uri.startswith("file://"):
        return parse.unquote(parse.urlparse(uri).path)
    else:
        raise RuntimeError(f"Invalid URI: {uri}")


def get_catkin_ws_paths() -> List[str]:
    """ホームディレクトリ直下のcatkinワークスペースまでのパスのリストを返す．"""
    catkin_tools_paths = glob(osp.expanduser("~/*/.catkin_tools/"))
    return [path.replace(".catkin_tools/", "") for path in catkin_tools_paths]


def get_catkin_ws_path(path: str) -> str:
    """ファイルが属するcatkinワークスペースの絶対パスを返す．"""
    path_ = osp.abspath(path)
    while path_ != "/":
        if osp.exists(osp.join(path_, ".catkin_tools/")):
            return path_
        path_ = osp.dirname(path_)
    else:
        raise RuntimeError(f"{path_} is not located under a catkin workspace.")


def is_in_catkin_src(path: str) -> bool:
    """パスがcatkinワークスペースのsrcディレクトリ以下に存在するかどうかを返す．"""
    # パスの存在を確認
    path_ = osp.abspath(path)
    if not osp.exists(path_):
        return False

    # catkinワークスペースのパスを取得
    try:
        ws_path = get_catkin_ws_path(path_)
    except:
        return False

    # srcディレクトリの存在を確認
    src_dir = osp.join(ws_path, "src/")
    if not osp.exists(src_dir):
        return False

    # srcそのものである場合と，srcディレクトリの内部に存在する場合にTrueを返す．
    return path == osp.join(ws_path, "src") or path_.startswith(src_dir)
