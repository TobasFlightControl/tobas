import os.path as osp
import urllib
import rospkg
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
        return urllib.parse.unquote(urllib.parse.urlparse(uri).path)
    else:
        raise RuntimeError(f"Invalid URI: {uri}")


def get_catkin_ws_paths() -> List[str]:
    """ホームディレクトリ直下のcatkinワークスペースまでのパスのリストを返す．"""
    catkin_tools_paths = glob(osp.expanduser("~/*/.catkin_tools/"))
    return [path.replace(".catkin_tools/", "") for path in catkin_tools_paths]


def get_catkin_ws_path(path_in: str) -> str:
    """ファイルが属するcatkinワークスペースのパスを返す．"""
    path = osp.abspath(path_in)
    while path != "/":
        if osp.exists(osp.join(path, ".catkin_tools/")):
            return path
        path = osp.dirname(path)
    else:
        raise RuntimeError(f"{path} is not located under a catkin workspace.")
