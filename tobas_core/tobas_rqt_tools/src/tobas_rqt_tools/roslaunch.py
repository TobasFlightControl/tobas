from subprocess import Popen
import roslaunch
from roslaunch import rlutil, parent
from typing import List, Dict, Optional


def rosrun(pkg_name: str, node_type: str, node_name: Optional[str] = None) -> Popen:
    command = ["rosrun", pkg_name, node_type]
    if node_name is not None:
        command += [f"__name:={node_name}"]

    # 別プロセスでrosrunを実行
    return Popen(command)


def launch(pkg_name: str, launch_name: str, args: Dict[str, str] = dict()) -> Popen:
    assert launch_name.endswith(".launch")

    command = ["roslaunch", pkg_name, launch_name]
    for key, value in args.items():
        command += [f"{key}:={value}"]

    # 別プロセスでroslaunchを実行
    return Popen(command)


def create_launcher(
    pkg_name: str, launch_name: str, args: List[str] = [], autostart: bool = True
) -> parent.ROSLaunchParent:
    """
    呼び出し元と同じプロセスでroslaunchを起動．
    NOTE: roslaunch.parent.ROSLaunchParentを使うとQWidget.closeEventが呼ばれなくなる．
    """
    assert launch_name.endswith(".launch")

    uuid = rlutil.get_or_generate_uuid(None, False)
    files = [(rlutil.resolve_launch_arguments([pkg_name, launch_name])[0], args)]

    roslaunch.configure_logging(uuid)
    launcher = parent.ROSLaunchParent(uuid, files)

    if autostart:
        launcher.start()

    return launcher
