from xml.etree import ElementTree as ET
from typing import Tuple


class TetherStationForceModel(ET.Element):
    def __init__(
        self,
        link_name: str,
        world_end: Tuple[float, float, float],
        drone_end: Tuple[float, float, float],
        tension: float,
    ) -> None:
        assert tension >= 0.0

        # robot/gazebo
        super().__init__("gazebo")

        # robot/gazebo/plugin
        plugin = ET.SubElement(self, "plugin")
        plugin.attrib["filename"] = "libtobas_gazebo_tether_station_force_plugin.so"
        plugin.attrib["name"] = "tobas_gazebo_tether_station_force_plugin"

        ET.SubElement(plugin, "linkName").text = link_name
        ET.SubElement(plugin, "worldEnd").text = " ".join(map(str, world_end))
        ET.SubElement(plugin, "droneEnd").text = " ".join(map(str, drone_end))
        ET.SubElement(plugin, "tension").text = str(tension)


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
    link_name: str,
    world_end: Tuple[float, float, float],
    drone_end: Tuple[float, float, float],
    tension: float,
):
    robot.append(TetherStationForceModel(link_name, world_end, drone_end, tension))
    robot.append(TetherStationVisualModel(link_name, world_end, drone_end))
