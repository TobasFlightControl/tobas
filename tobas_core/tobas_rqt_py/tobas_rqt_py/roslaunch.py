from subprocess import Popen
from typing import Dict, Optional


def rosrun(pkg_name: str, node_type: str, node_name: Optional[str] = None) -> Popen:
    command = ["ros2", "run", pkg_name, node_type]
    if node_name is not None:
        command += ["--ros-args", "--remap", f"__node:={node_name}"]

    # 別プロセスでrosrunを実行
    return Popen(command)


def launch(pkg_name: str, launch_name: str, args: Dict[str, str] = dict()) -> Popen:
    assert launch_name.endswith(".launch")

    command = ["ros2", "launch", pkg_name, launch_name]
    for key, value in args.items():
        command += [f"{key}:={value}"]

    # 別プロセスでroslaunchを実行
    return Popen(command)
