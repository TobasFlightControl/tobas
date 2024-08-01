from xml.etree import ElementTree as ET
from typing import Tuple

from tobas_urdf_tools_py.core import Joint
from tobas_urdf_tools_py.dummy import DummyVisualLink


class TetherStationForceModel(ET.Element):
    def __init__(
        self,
        ns: str,
        link_name: str,
        world_end: Tuple[float, float, float],
        drone_end: Tuple[float, float, float],
        init_tension: float,
        init_max_length: float,
        young_modulus: float,
        cross_section_area: float,
    ) -> None:
        assert init_tension >= 0.0
        assert init_max_length > 0.0
        assert young_modulus > 0.0
        assert cross_section_area > 0.0

        # robot/gazebo
        super().__init__("gazebo")

        # robot/gazebo/plugin
        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "libtobas_gazebo_tether_station_force_plugin.so"
        plugin.attrib["name"] = "tobas_gazebo_tether_station_force_plugin"

        ET.SubElement(plugin, "robotNamespace").text = ns
        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "worldEnd").text = " ".join(map(str, world_end))
        ET.SubElement(plugin, "droneEnd").text = " ".join(map(str, drone_end))
        ET.SubElement(plugin, "initialTension").text = str(init_tension)
        ET.SubElement(plugin, "initialMaximumLength").text = str(init_max_length)
        ET.SubElement(plugin, "youngModulus").text = str(young_modulus)
        ET.SubElement(plugin, "crossSectionArea").text = str(cross_section_area)


class TetherStationVisualModel(ET.Element):
    def __init__(
        self,
        link_name: str,
        world_end: Tuple[float, float, float],
        drone_end: Tuple[float, float, float],
    ) -> None:

        # robot/gazebo
        super().__init__("gazebo", reference=link_name)

        # robot/gazebo/visual
        visual = ET.SubElement(self, "visual")

        # robot/gazebo/visual/plugin
        plugin = ET.SubElement(visual, "plugin")
        plugin.attrib["filename"] = "libtobas_gazebo_tether_station_visual_plugin.so"
        plugin.attrib["name"] = "tobas_gazebo_tether_station_visual_plugin"

        ET.SubElement(plugin, "worldEnd").text = " ".join(map(str, world_end))
        ET.SubElement(plugin, "droneEnd").text = " ".join(map(str, drone_end))


def add_tether_station_model(
    robot: ET.Element,
    ns: str,
    link_name: str,
    world_end: Tuple[float, float, float],
    drone_end: Tuple[float, float, float],
    init_tension: float,
    init_max_length: float,
    young_modulus: float,
    cross_section_area: float,
) -> None:
    # Plugins
    robot.append(
        TetherStationForceModel(
            ns=ns,
            link_name=link_name,
            world_end=world_end,
            drone_end=drone_end,
            init_tension=init_tension,
            init_max_length=init_max_length,
            young_modulus=young_modulus,
            cross_section_area=cross_section_area,
        )
    )
    robot.append(TetherStationVisualModel(link_name=link_name, world_end=world_end, drone_end=drone_end))

    # VisualPluginが埋め込まれたリンク名で始まるリンクに，Visualタグが設定されている必要がある．
    dummy_link_name = f"{link_name}_dummy_visual_link"
    dummy_joint_name = f"{link_name}_dummy_visual_joint"
    robot.append(DummyVisualLink(dummy_link_name))
    robot.append(Joint(name=dummy_joint_name, type="fixed", parent=link_name, child=dummy_link_name))
