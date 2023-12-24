from xml.etree import ElementTree as ET

from tobas_urdf_tools_py.core import *


class BaseStaticJoint(ET.Element):
    def __init__(self, root_link: str):
        assert root_link != "world"

        super().__init__("xacro:if", value="$(arg DEBUG)")
        self.append(Link(name="world"))
        self.append(
            Joint(
                name="base_static_joint", type="fixed", parent="world", child=root_link
            )
        )
