from xml.etree import ElementTree as ET

from .noise import Noise


class GazeboRosControl(ET.Element):
    def __init__(self, namespace: str) -> None:
        super().__init__("gazebo")

        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "libgazebo_ros_control.so"
        plugin.attrib["name"] = "gazebo_ros_control"

        ET.SubElement(plugin, "robotNamespace").text = namespace
        ET.SubElement(plugin, "robotSimType").text = "gazebo_ros_control/DefaultRobotHWSim"
        ET.SubElement(plugin, "legacyModeNS").text = "true"


class Camera(ET.Element):
    RGB8 = "R8G8B8"
    L8 = "L8"

    def __init__(
        self, width: int, height: int, near: float, far: float, fov: float, format: str = RGB8, noise: Noise = None
    ) -> None:
        assert width > 0
        assert height > 0
        assert 0.0 < near < far
        assert fov > 0.0

        super().__init__("camera")

        # camera/image
        image = ET.SubElement(self, "image")
        ET.SubElement(image, "format").text = format
        ET.SubElement(image, "width").text = f"{width}"
        ET.SubElement(image, "height").text = f"{height}"

        # camera/clip
        clip = ET.SubElement(self, "clip")
        ET.SubElement(clip, "near").text = f"{near}"
        ET.SubElement(clip, "far").text = f"{far}"

        # camera/horizontal_fov
        ET.SubElement(self, "horizontal_fov").text = f"{fov}"

        # camera/noise
        if noise:
            self.append(noise)
