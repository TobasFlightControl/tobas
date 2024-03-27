from subprocess import Popen
import roslaunch
from roslaunch import rlutil, parent
from typing import List


def create_launcher(
    pkg_name: str, launch_name: str, args: List[str] = [], autostart: bool = True
) -> parent.ROSLaunchParent:
    assert launch_name.endswith(".launch")

    uuid = rlutil.get_or_generate_uuid(None, False)
    files = [(rlutil.resolve_launch_arguments([pkg_name, launch_name])[0], args)]

    roslaunch.configure_logging(uuid)
    launcher = parent.ROSLaunchParent(uuid, files)

    if autostart:
        launcher.start()

    return launcher


def rosrun(pkg_name: str, node_type: str, node_name: str = None) -> Popen:
    command = ["rosrun", pkg_name, node_type]
    if node_name is not None:
        command += [f"__name:={node_name}"]

    # バックグラウンドでrosrunを実行
    return Popen(command)
