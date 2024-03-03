import os.path as osp
import urllib
import rospkg


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
