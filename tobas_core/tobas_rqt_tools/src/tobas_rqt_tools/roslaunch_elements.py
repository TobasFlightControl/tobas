from xml.etree import ElementTree as ET
from typing import Optional


def _bool2str(x: bool) -> str:
    return "true" if x else "false"


class Launch(ET.Element):
    def __init__(self) -> None:
        super().__init__("launch")

    def add_arg(self, name: str, default: str) -> None:
        arg = ET.SubElement(self, "arg")
        arg.attrib["name"] = name
        arg.attrib["default"] = default


class Param(ET.Element):
    def __init__(self, name: str, command: str) -> None:
        super().__init__("param", name=name, command=command)


class RosParam(ET.Element):
    LOAD = "load"

    def __init__(self, file: str, command: str = LOAD) -> None:
        super().__init__("rosparam", file=file, command=command)


class Node(ET.Element):
    SCREEN = "screen"
    LOAD = "load"

    def __init__(
        self,
        pkg: str,
        type: str,
        name: str,
        output: str = SCREEN,
        required: bool = False,
        respawn: bool = False,
        ns: Optional[str] = None,
        args: Optional[str] = None,
    ) -> None:
        super().__init__(
            "node",
            pkg=pkg,
            type=type,
            name=name,
            output=output,
            required=_bool2str(required),
            respawn=_bool2str(respawn),
        )

        if ns:
            self.attrib["ns"] = ns

        if args:
            self.attrib["args"] = args

    def add_remap(self, from_: str, to: str) -> None:
        remap = ET.SubElement(self, "remap")
        remap.attrib["from"] = from_
        remap.attrib["to"] = to

    def add_rosparam(self, file: str, command: str = LOAD) -> None:
        rosparam = ET.SubElement(self, "rosparam")
        rosparam.attrib["file"] = file
        rosparam.attrib["command"] = command


class Include(ET.Element):
    def __init__(self, file: str) -> None:
        assert file.endswith(".launch")

        super().__init__("include", file=file)

    def add_arg(self, name: str, value: str) -> None:
        arg = ET.SubElement(self, "arg")
        arg.attrib["name"] = name
        arg.attrib["value"] = value
